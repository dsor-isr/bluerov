"""
Original source code from: https://gist.github.com/ES-Alexander
Author: ES-Alexander
If you use this library, please follow the original author at: https://github.com/ES-Alexander
and give this wonderfull human being some well deserved love 🤓
"""
import time
from numbers import Number
from itertools import repeat, chain
from .mavactive import mavactive, mavutil, mavlink
from functools import partial
import rospy

# logging setup
import logging

logging.NOTE = logging.INFO - 5
logging.addLevelName(logging.NOTE, 'NOTE')
class NoteLogger(logging.getLoggerClass()):
    def note(self, msg, *args, **kwargs):
        if self.isEnabledFor(logging.NOTE):
            self._log(logging.NOTE, msg, args, **kwargs)

logging.setLoggerClass(NoteLogger)
logging.note = partial(NoteLogger.note, logging.getLogger())

def configure_logging(level, filename, datefmt):
    # configure logging, based on defaults/user input
    log_level = getattr(logging, level)
    logging.basicConfig(filename=filename, level=log_level, datefmt=datefmt,
                        format=('%(levelname)s: %(asctime)s.%(msecs)03d - '
                                '%(message)s'))

rospy.loginfo("1. Started autopilot instance")
rospy.loginfo("2. Heartbeat was sent to the pixhawk")


class Autopilot:
    """ An ArduSub autopilot connection manager. """
    def __init__(self, *args, client=True, thrusters=8, **kwargs):
        self.thrusters = thrusters
        self.master = mavutil.mavlink_connection(*args, **kwargs)

        if client:
            logging.debug('connecting to MAVLink client...')
            self.master.wait_heartbeat()
        else:
            logging.debug('connecting to MAVLink server...')
            self._server_wait_conn()

        # send regular heartbeats, as a ground control station
        self.heart = mavactive(self.master, mavlink.MAV_TYPE_GCS)

        # convenience
        self.mav        = self.master.mav
        self.recv_match = self.master.recv_match
        self.target     = (self.master.target_system,
                           self.master.target_component)

        logging.info('MAVLink connection successful')

    def _server_wait_conn(self):
        while 'waiting for server to respond':
            self.master.mav.ping_send(int(time.time() * 1e6), 0, 0, 0)
            if self.master.recv_match():
                break
            time.sleep(0.5)

    def read_param(self, name: str, index: int=-1, timeout: float=None):
        self.mav.param_request_read_send(
            *self.target,
            name.encode('utf8'),
            index
        )
        logging.note(f'read_param({name=}, {index=}, {timeout=})')
        return self.recv_match(type='PARAM_VALUE', blocking=True,
                               timeout=timeout)

    def set_param(self, name: str, value: float, type: int=0,
                  timeout: float=1, retries: int=3):
        name = name.encode('utf8')
        self.mav.param_set_send(
            *self.target,
            name, value, type
        )
        logging.info(f'set_param({name=}, {value=}, {type=})')

        while not (msg := self.recv_match(type='PARAM_VALUE', blocking=True,
                                          timeout=timeout)) and retries > 0:
            retries -= 1
            logging.debug(f'param set timed out after {timeout}s, retrying...')
            self.mav.param_set_send(
                *self.target,
                name, value, type
            )
        return msg

    def request_message_interval(self, message_id: int, frequency_hz: float):
        """
        Request MAVLink message in a desired frequency,
        documentation for SET_MESSAGE_INTERVAL:
            https://mavlink.io/en/messages/common.html#MAV_CMD_SET_MESSAGE_INTERVAL

        Args:
            message_id (int): MAVLink message ID
            frequency_hz (float): Desired frequency in Hz
        """
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            message_id, # The MAVLink message ID
            1e6 / frequency_hz, # The interval between two messages in microseconds. Set to -1 to disable and 0 to request default rate.
            0, 0, 0, 0, # Unused parameters
            0, # Target address of message stream (if message has target address fields). 0: Flight-stack default (recommended), 1: address of requestor, 2: broadcast.
        )

    def look_at(self, tilt, roll=0, pan=0):
        """
        Moves gimbal to given position
        Args:
            tilt (float): tilt angle in degrees (0 is forward)
        """
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
            1, 
            tilt*100, roll*100, pan*100,
            0, 0, 0,
            mavutil.mavlink.MAV_MOUNT_MODE_MAVLINK_TARGETING)

    def __enter__(self):
        ''' Send regular heartbeats while using in a context manager. '''
        logging.info('__enter__ -> reviving heart (if required)')
        self.heart.revive()
        return self

    def __exit__(self, *args):
        ''' Automatically disarm and stop heartbeats on error/context-close. '''
        logging.info('__exit__ -> disarming, and stopping heart')
        try:
            self.disarm()
        except OSError:
            raise
            pass
        self.heart.kill()

    def arm(self):
        self.master.arducopter_arm()
        logging.debug('arm requested, waiting...')
        self.master.motors_armed_wait()
        logging.info('Motors armed!')

    def disarm(self):
        self.master.arducopter_disarm()
        logging.debug('disarm requested, waiting...')
        self.master.motors_disarmed_wait()
        logging.info('Motors disarmed')

    def set_mode(self, mode):
        ''' Sets autopilot 'mode', by name or id number. '''
        if isinstance(mode, str):
            for attempt in range(5):
                if (mapping := self.master.mode_mapping()):
                    break # success - mapping available
                self.master.wait_heartbeat()
                logging.info('mode mapping not available, retrying...')
            else:
                logging.warning('mode change failed - no mapping available!')
                return
            mode_id = mapping[mode.upper()]
        else:
            mode_id = mode
        self.master.set_mode(mode_id)
        logging.debug(f'setting {mode=} ({mode_id=}), waiting...')

        while 'mode change not confirmed':
            ack_msg = self.recv_match(type='COMMAND_ACK', blocking=True)
            # check if acknowledged MAV_CMD_DO_SET_MODE or SET_MODE (old)
            if ack_msg.command in (176, 11):
                if ack_msg.result == 0:
                    logging.info(f'{mode=}, change successful')
                else:
                    result = mavlink.enums['MAV_RESULT'][ack_msg.result]
                    logging.warning('mode change failed!\n\t%s: %s',
                                    result.name, result.description)
                break

    def set_servo(self, servo, pwm):
        ''' Set a single servo output pwm value.
        'servo' can only be outputs that aren't assigned as motors, so is
          generally used for lights/camera etc.
        When in a per_thruster_control context in Servo mode, also allows
          controlling individual thrusters.
        '''
        logging.info(f'set_servo({servo=}, {pwm=})')
        self.master.set_servo(servo, pwm)

    def send_rc(self, rcin1=65535, rcin2=65535, rcin3=65535, rcin4=65535,
                rcin5=65535, rcin6=65535, rcin7=65535, rcin8=65535,
                rcin9=65535, rcin10=65535, rcin11=65535, rcin12=65535,
                rcin13=65535, rcin14=65535, rcin15=65535, rcin16=65535,
                rcin17=65535, rcin18=65535,
                pitch=None, roll=None, throttle=None, yaw=None, forward=None,
                lateral=None, camera_pan=None, camera_tilt=None, lights1=None,
                lights2=None, video_switch=None):
        ''' Sets all 18 rc channels as specified.
        Values should be between 1100-1900, or left as 65535 to ignore.
        Can specify values:
            positionally,
            or with rcinX (X=1-18),
            or with default RC Input channel mapping names
              -> see https://ardusub.com/developers/rc-input-and-output.html
        It's possible to mix and match specifier types (although generally
          not recommended). Default channel mapping names override positional
          or rcinX specifiers.
        '''
        rc_channel_values = (
            pitch        or rcin1,
            roll         or rcin2,
            throttle     or rcin3,
            yaw          or rcin4,
            forward      or rcin5,
            lateral      or rcin6,
            camera_pan   or rcin7,
            camera_tilt  or rcin8,
            lights1      or rcin9,
            lights2      or rcin10,
            video_switch or rcin11,
            rcin12, rcin13, rcin14, rcin15, rcin16, rcin17, rcin18
        )
        logging.info(f'send_rc')
        logging.debug(rc_channel_values)
        self.mav.rc_channels_override_send(
            *self.target,
            *rc_channel_values
        )

    def clear_motion(self, stopped_pwm=1500):
        ''' Sets all 8 motion direction RC inputs to 'stopped_pwm'. '''
        logging.info('clear_motion')
        self.send_rc(*[stopped_pwm]*8)

    def get_thruster_outputs(self):
        ''' Returns (and notes) the first 'self.thrusters' servo PWM values.
        Offset by 1500 to make it clear how each thruster is active.
        '''
        logging.debug('get_thruster_outputs')
        servo_outputs = self.recv_match(type='SERVO_OUTPUT_RAW',
                                        blocking=True).to_dict()
        thruster_outputs = [servo_outputs[f'servo{i+1}_raw'] - 1500
                            for i in range(self.thrusters)]
        logging.note(f'{thruster_outputs=}')
        return thruster_outputs

    def status_loop(self, duration, delay=0.05):
        ''' Loop for 'duration', with 'delay' between iterations. [s]
        Useful for debugging.
        '''
        start = time.time()
        while time.time() - start < duration:
            self.get_thruster_outputs()
            time.sleep(delay)

    def command_long_send(self, command, confirmation=0, param1=0, param2=0,
                          param3=0, param4=0, param5=0, param6=0, param7=0):
        self.mav.command_long_send(
            *self.target,
            getattr(mavlink, f'MAV_CMD_{command}', command),
            confirmation,
            param1, param2, param3, param4, param5, param6, param7
        )

    def set_message_interval(self, message, interval, response_target=0):
        ''' Set message interval, as per MAV_CMD_SET_MESSAGE_INTERVAL.
        Required to get specific messages if not auto-streamed by connection.
        'message' is the name or id of the message
        'interval' is the desired interval [us]
        '''
        logging.info(f'set_message_interval({message=}, {interval=})')
        self.command_long_send(
            'SET_MESSAGE_INTERVAL',
            param1 = getattr(mavlink, f'MAVLINK_MSG_ID_{message}', message),
            param2 = interval, param7 = response_target)

    def set_bulk_message_intervals(self, messages, interval=None):
        ''' Set several message intervals at once. '''
        if interval is None:
            if isinstance(messages, dict):
                iterator = messages.items()
            else:
                # assume iterator of pairs
                iterator = messages
        elif isinstance(interval, Number):
            iterator = zip(messages, repeat(interval))
        else:
            iterator = zip(messages, interval)

        for message, interval in iterator:
            self.set_message_interval(message, interval)

    def disconnect(self):
        ''' End the MAVLink connection. '''
        logging.info('disconnect -> closing MAVLink connection')
        self.master.close()

    def __del__(self):
        ''' Clean up if/when autopilot object is deleted. '''
        self.__exit__(None, None, None)
        self.disconnect()

    @classmethod
    def connect_to_client(cls, ip='0.0.0.0', port=14550, *args, **kwargs):
        ''' Default for topside connection to Blue Robotics Companion 0.0.x '''
        logging.note(f'connect_to_client({ip=}, {port=}, {args=}, {kwargs=})')
        return cls(f'udpin:{ip}:{port}', *args, **kwargs)

    @classmethod
    def connect_to_server(cls, ip='companion.local', port=14550, *args, **kwargs):
        ''' Default for topside connection to Blue Robotics Companion 1.x.y '''
        logging.note(f'connect_to_server({ip=}, {port=}, {args=}, {kwargs=})')
        return cls(f'udpout:{ip}:{port}', *args, **kwargs, client=False)

