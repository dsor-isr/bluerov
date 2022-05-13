"""
Original source code from: https://gist.github.com/ES-Alexander
Author: ES-Alexander
If you use this library, please follow the original author at: https://github.com/ES-Alexander
and give this wonderfull human being some well deserved love 🤓
"""
from .Autopilot import Autopilot
from contextlib import contextmanager
from itertools import repeat, chain

import logging
from functools import partial

logging.NOTE = logging.INFO - 5
logging.addLevelName(logging.NOTE, 'NOTE')
class NoteLogger(logging.getLoggerClass()):
    def note(self, msg, *args, **kwargs):
        if self.isEnabledFor(logging.NOTE):
            self._log(logging.NOTE, msg, args, **kwargs)

logging.setLoggerClass(NoteLogger)
logging.note = partial(NoteLogger.note, logging.getLogger())

class ExtendedAutopilot(Autopilot):
    ''' An Autopilot with extended control functionality. '''
    SERVO_OUTPUT_FUNCTIONS = {
        'Disabled': 0,
        'RCPassThru': 1,
        **{f'Motor{i}': param_value for i, param_value in
           enumerate(chain(range(33, 33+8), range(82, 82+4)), start=1)},
        **{f'RCIN{i}': param_value for i, param_value in
           enumerate(range(51, 51+16), start=1)},
    }

    @contextmanager
    def per_thruster_control(self, mode='Servo', stopped_pwm=1500,
                             backup_timeout: float=None,
                             set_timeout: float=1, set_retries: int=3):
        ''' Allow thrusters to be controlled individually, within a context.
        WARNING: while in context, thruster actions no longer require arming,
          and failsafes are no longer engaged!
        'mode' can be 'Servo' (default) or 'RCPassThru' to control thrusters
          either using self.set_servo, or using self.send_rc.
        WARNING: if the vehicle is powered down while in RCPassThru mode the
          thrusters may become active WITHOUT ARMING on next power up, from
          joystick input. It is more powerful than Servo mode because multiple
          thrusters can be controlled in a single function call, but use with
          significant caution.
        For vehicles with more than 6 thrusters RCPassThru mode is not
          recommended, because it doubles-up on existing RC Input mappings
          (e.g. Camera Pan and Tilt are mapped to inputs 7 and 8). That issue
          can be worked around if necessary by mapping individual outputs to
          only otherwise unused RCINx inputs (generally 1-6, 12-18), but that's
          not currently supported/implemented by this function.
        Another alternative is remapping the input channels with PARAM_MAP_RC,
          although that can cause some difficult debugging issues when
          comparing behavior to BR documentation/examples, which assumes the
          default mappings.
        '''
        logging.info('Engaging per-thruster control!')
        logging.warning('Thruster actions no longer require arming!')
        logging.warning('Failsafes no longer engaged!')

        if mode == 'RCPassThru':
            output_function = self.SERVO_OUTPUT_FUNCTIONS[mode]
            logging.critical('RCPassThru mode engaged:\n'
                ' -> THRUSTERS RESPOND DIRECTLY TO RC/JOYSTICK INPUTS')
        elif mode == 'Servo':
            output_function = self.SERVO_OUTPUT_FUNCTIONS['Disabled']
            logging.info('Servo mode engaged:\n'
                         ' -> control thrusters with self.set_servo')
        else:
            raise ValueError(f'Invalid per-thruster control {mode=}')

        self._backup_servo_functions(backup_timeout)
        try:
            # try to make sure thrusters are stopped before changeover
            self._stop_thrusters(mode, stopped_pwm)
            # set first n SERVO outputs to passthrough first n RC inputs
            for n in range(1, self.thrusters + 1):
                self.set_param(f'SERVO{n}_FUNCTION', output_function,
                               timeout=set_timeout, retries=set_retries)
            # make sure thrusters are actually stopped
            self._stop_thrusters(mode, stopped_pwm)

            yield self
        finally:
            # make sure to stop thrusters before changing back
            self._stop_thrusters(mode, stopped_pwm)
            self._restore_servo_functions(set_timeout, set_retries)

        logging.info('per-thruster control successfully exited')
        logging.warning('Arming and failsafe requirements re-engaged')

    def _stop_thrusters(self, mode, stopped_pwm):
        if mode == 'RCPassThru':
            self.send_rc(*[stopped_pwm]*self.thrusters)
        else:
            for servo in range(1, self.thrusters + 1):
                
                # Hardcoded for the version with 8 thrusters (HEAVY) # TODO - improve this bit
                if servo != 9:
                    self.set_servo(servo, stopped_pwm)

                # Hardcoded here for the lights - #TODO - to be fixed
                if servo == 9:
                    self.set_servo(servo, 1100)


    def _backup_servo_functions(self, timeout: float=None):
        logging.note('backing up servo thruster functions')
        self._old_servo_functions = {}
        for n in range(1, self.thrusters + 1):
            param = f'SERVO{n}_FUNCTION'
            self._old_servo_functions[param] = self.read_param(param)

    def _restore_servo_functions(self, timeout: float=1, retries: int=3):
        for param, backup in self._old_servo_functions.items():
            self.set_param(param, backup.param_value, backup.param_type,
                           timeout, retries)
        logging.note('servo functions restored')