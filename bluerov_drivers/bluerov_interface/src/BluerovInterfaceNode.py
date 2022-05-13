#!/usr/bin/env python3

'''
File name: BluerovInterfaceNode.py
Author: Marcelo Jacinto and Andre Potes
Date created: 24/02/2022
Date last modified: 21/03/2022
Python Version: 3.8
'''

import rospy
from pymavlink import mavutil
from mavlink_lib.ExtendedAutopilot import ExtendedAutopilot
from std_msgs.msg import Float64, Bool
from dsor_msgs.msg import Measurement, Thruster
from dsor_msgs.srv import DoubleValue, DoubleValueResponse
from medusa_msgs.msg import Pressure
from sensor_msgs.msg import NavSatFix

class BluerovInterfaceNode:

    def __init__(self, autopilot: ExtendedAutopilot, num_thrusters: int):
        """
        Constructor of the ROS node
        """
        # Define the method to run on ros shutdown
        rospy.on_shutdown(self.on_ros_shutdown)

        # Load the parameters from the parameter server
        self.loadParams()

        # Save the autopilot object
        self.autopilot = autopilot
        self.num_thrusters = num_thrusters

        # Set the thruster input to be zeros (aka 1500 PWM)
        self.thrust_input = [1500] * self.num_thrusters

        # Variable that determines if we want to kill the thrusters
        self.last_thrusters_safety = rospy.Time.now()
        self.thrusters_safety = False

        # Initialize the mavlink connection
        self.initializeMavlink()

        # Initialize all the ROS communication side
        self.initializePublishers()
        self.initializeSubscribers()
        self.initializeServices()

        # Initialize the watchdog variable for the thrusters 
        self.last_update_thruster_time = rospy.Time().now()

        # start the ROS NODE update callback
        self.initializeTimer()

    def loadParams(self):
        """
        Method to load configuration parameters from the ros parameter server
        """

        # Update frequency of the node
        self.node_frequency: float = rospy.get_param('~node_frequency', default=10.0)

        # Constant to convert the pressure from the pressure sensor to depth in [m]
        self.depth_atm_pressure: float = rospy.get_param('~depth_atm_pressure', default=10.1452685894133) # 1.028*10.19977334

        # Get the watchdog for the thrusters in seconds
        self.thrusters_watchdog_const = rospy.get_param('~watchdog_thrusters', default=2.0) # we must receive references for the thrusters, otherwise they stop 

        # Get the thrusters stafety timeout in seconds
        self.thrusters_safety_const = rospy.get_param('~safety_thrusters', default=1.0) # we must receive a bool from the topic /thrusters_safety otherwise they wont rotate 

        # Battery tunning
        self.battery_offset: float = rospy.get_param('~battery_offset', default=0.0)
        self.battery_gain: float = rospy.get_param('~battery_gain', default=1.0)

    def initializeMavlink(self):
        """
        Method used to initiate the mavlink connection and allow for vehicle telemetry and control
        """

        # request SERVO_OUTPUT_RAW at 20Hz
        self.autopilot.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, 10)

        # request sensor telemetry to be delivered at the same rate this node is running at
        self.autopilot.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 10)            # IMU
        self.autopilot.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_SCALED_PRESSURE2, 10)    # EXTERNAL DEPTH SENSOR
        self.autopilot.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, 10)         # GPS
        self.autopilot.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 10) # 
        self.autopilot.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_RANGEFINDER, 10)         # ALTIMTETER

        # make sure all motion is set to stationary before arming and only then arm the vehicle
        self.autopilot.clear_motion()

    def on_ros_shutdown(self):
        """
        Callback that is called upon noe termination (or Ctrl+C)
        """
        # For now, there is nothing to be done at node shutdown
        pass

    def initializePublishers(self):
        """
        Method that initializes the ROS publishers (to publish the sensor data to the control and navigation stack)
        """
        
        # IMU data publisher
        self.imu_pub = rospy.Publisher(rospy.get_param('~topics/publishers/imu'), Measurement, queue_size=1)
        self.imu_pub_data = rospy.Publisher(rospy.get_param('~topics/publishers/data/imu'), Measurement, queue_size=1)

        # Altimiter data publisher
        self.altimeter_pub = rospy.Publisher(rospy.get_param('~topics/publishers/altimeter'), Measurement, queue_size=1)
        self.altimeter_pub_data = rospy.Publisher(rospy.get_param('~topics/publishers/data/altimeter'), Measurement, queue_size=1)

        # Depth sensor data publisher
        self.depth_pub = rospy.Publisher(rospy.get_param('~topics/publishers/depth'), Measurement, queue_size=1)
        self.depth_pub_data = rospy.Publisher(rospy.get_param('~topics/publishers/data/depth'), Measurement, queue_size=1)
        self.pressure_pub = rospy.Publisher(rospy.get_param('~topics/publishers/data/pressure'), Pressure, queue_size=1)

        # GPS data publisher
        self.gps_pub = rospy.Publisher(rospy.get_param('~topics/publishers/gps'), NavSatFix, queue_size=1)

        # Voltage and current publisher
        self.voltage_pub = rospy.Publisher(rospy.get_param('~topics/publishers/data/voltage'), Float64, queue_size=1)
        self.current_pub = rospy.Publisher(rospy.get_param('~topics/publishers/data/current'), Float64, queue_size=1)
    
    def initializeSubscribers(self):
        """
        Method that initializes the ROS subscribers (to receive the thrust to apply to the vehicle and the desired position of the gimbal)
        """

        # Subscribe to thrusters control input
        rospy.Subscriber(rospy.get_param('~topics/subscribers/thrusters'), Thruster, self.thrusters_callback)	

        # Subscribe to the camera gimbal angle
        rospy.Subscriber(rospy.get_param('~topics/subscribers/gimbal'), Float64, self.gimbal_callback)

        # Subscribe to the Kill switch
        rospy.Subscriber(rospy.get_param('~topics/subscribers/thrusters_safety'), Bool, self.thrusters_safety_callback)

    def initializeServices(self):
        """
        Method that initializes all the services (lights in this case)
        """

        # Service server for controlling the lights of the vehicle
        self.lights_srv = rospy.Service(rospy.get_param('~topics/services/lights'), DoubleValue, self.lights_service_callback)

    def initializeTimer(self):
        """
        Method that starts the system timer that periodically calls a callback
        """
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.node_frequency), self.timerCallback)

    # -----------------
    # Callbacks section
    # -----------------
    def timerCallback(self, event):

        # Check if we have not received from thrusters safety for more than X seconds
        if ((rospy.Time.now() - self.last_thrusters_safety).to_sec() > self.thrusters_safety_const):
            self.thrusters_safety = False

        # 0. Check the watchdog for the thrusters - if no communication was obtained for a given period of time stop the thrusters!
        if (rospy.Time().now() - self.last_update_thruster_time).to_sec() > self.thrusters_watchdog_const or self.thrusters_safety == False:
            self.thrust_input = [1500] * self.num_thrusters

        # 0.1 Update the thrusters with the desired PWM signal
        for i in range(0, self.num_thrusters):
            self.autopilot.set_servo(i + 1, self.thrust_input[i])
        
        # Auxiliary variables for receiving the data
        data = 1
        gps_data_raw = None
        gps_data = None

        while data is not None:
            
            # Read the data from mavlink
            data = self.autopilot.master.recv_match()

            # Check if something was read, or we got nothing
            if data:
                
                # Convert to dictionary
                data = data.to_dict()
                
                # 1. Parse IMU data
                if data["mavpackettype"] == "ATTITUDE":
                    self.imu_to_ros(data)

                # 2. Parse Altimeter data
                if data["mavpackettype"] == "RANGEFINDER":
                    self.altimeter_to_ros(data)

                # 3. Parse depth data
                if data["mavpackettype"] == "SCALED_PRESSURE2":
                    self.depth_to_ros(data)

                # 4. Parse gps data (must parse two packets to get the number of satelites and check if the data is good)
                if data["mavpackettype"] == "GPS_RAW_INT":
                    gps_data_raw = data
                
                if data["mavpackettype"] == "GLOBAL_POSITION_INT":
                    gps_data = data

                if gps_data_raw and gps_data:

                    # Parse the actual GPS data 
                    self.gps_to_ros(gps_data_raw, gps_data)
                    
                    # Reset the GPS variables
                    gps_data_raw = None
                    gps_data = None

                # 5. Parse battery
                if data["mavpackettype"] == "SYS_STATUS":
                    self.battery_to_ros(data)
        

    def thrusters_callback(self, msg: Thruster):
        
        # Check if we have the correct number of thrusters in the message
        if len(msg.value) != self.num_thrusters:
            rospy.WARN("Vehicle driver only received commands for " + str(len(msg.value)) + " thrusters. NOT APPLYING FORCE")

        # Update the last time a thruster message was received
        self.last_update_thruster_time = rospy.Time.now()

        # NOTE - the input for the pixhawk must be between 1100 (max negative) and 1900 (max positive). The motors do not
        # spin at 1500. Therefore, we must scale the inputs that we receive from the medusa stack that are between -100 and 100!

        # Mask to rotate the motors if they are installed inverted
        mask = [1.0, 1.0, 1.0, 1.0, -1.0, 1.0, 1.0, -1.0]

        # for i in range(0, 5):
        #   self.thrust_input[i] = (msg.value[i] * mask[i] * 4) + 1500
        # self.thrust_input[5] = 0.0
        # self.thrust_input[6] = 0.0
        # self.thrust_input[7] = (msg.value[5] * mask[5] * 4) + 1500

        # Set the servo speeds according to the desired thrust
        for i in range(0, self.num_thrusters):
            self.thrust_input[i] = (msg.value[i] * mask[i] * 4) + 1500


    def gimbal_callback(self, msg: Float64):
        """
        Callback to control the gimbal angle (servo motor) of the camera
        """

        angle_deg = msg.data

        # Saturate the angle of the camera between -45 and 45 degrees
        if angle_deg > 90:
            angle_deg = 90
        elif angle_deg < -80:
            angle_deg = -80

        # Send the control command to the pixhawk
        self.autopilot.look_at(angle_deg)


    def thrusters_safety_callback(self, msg: Bool):
        """
        Callback to make sure we can activate the thrusters
        """
        self.thrusters_safety = msg.data
        self.last_thrusters_safety = rospy.Time.now()

    
    def lights_service_callback(self, req):
        """
        Service callback to control the lights of the vehicle
        """

        response = DoubleValueResponse()
        light_value = float(req.value)

        if 0 <= light_value <= 8:
            # NOTE - the input must be between 1100 (off) of 1900 (full-brightness)
            # Note that the lights is controlled by PWM like the thrusters, and it is connected
            # to the port next to the last thruster, hence can be controlled by the same driver
            # and using the next channel after the last thruster
            self.autopilot.set_servo(self.num_thrusters + 1, 1100 + (light_value*100))
            #rospy.loginfo("Setting lights to level: " + str(light_value))
            response.res = True
            return response
        
        rospy.loginfo("Lights level should be between 0 (OFF) and 8 (FULL-BRIGHTNESS)")
        response.res = False
        return response

    # -----------------
    # Section that receives mavlink data and sends to ROS
    # -----------------
    def imu_to_ros(self, data):
        """
        Method that receives a mavlink message of ATTITUDE in dictionary format
        and publishes the data to ROS topic
        """

        imu_mesurement = Measurement()
        imu_mesurement.header.frame_id = 'bluerov_ahrs'
        imu_mesurement.header.stamp = rospy.Time.now()

        # Use the orientation and angular-rates data
        imu_mesurement.value = [data['roll'], 
                                data['pitch'], 
                                data['yaw'], 
                                data['rollspeed'], 
                                data['pitchspeed'], 
                                data['yawspeed']]

        # Publish the imu message for the filter topic and the drivers topic
        self.imu_pub.publish(imu_mesurement)
        self.imu_pub_data.publish(imu_mesurement)
        
    def battery_to_ros(self, data):
        """
        Method that receives a mavlink message of SYS_STATUS in dictionary format
        and publishes the data to the ROS topic
        """
        voltage = Float64()
        current = Float64()
        voltage.data = self.battery_offset + (self.battery_gain * data["voltage_battery"] * 0.001)
        current.data = data["current_battery"] * 0.001

        # Publish the battery messages
        self.voltage_pub.publish(voltage)
        self.current_pub.publish(current)


    def altimeter_to_ros(self, data):
        """
        Method that receives a mavlink message of RANGEFINDER in dictionary format
        and publishes the data to ROS topic
        """

        altimeter_measurement = Measurement()
        altimeter_measurement.header.frame_id = 'bluerov_altimeter'
        altimeter_measurement.header.stamp = rospy.Time.now()

        # Use the range measured by the altimeter sensor
        altimeter_measurement.value = [data['distance']]

        # Publish the altimeter message to the filter topic and to the drivers topic
        self.altimeter_pub.publish(altimeter_measurement)
        self.altimeter_pub_data.publish(altimeter_measurement)

    def depth_to_ros(self, data):
        """
        Method that receives a mavlink message of SCALED_PRESSURE2 in dictionary format
        and publishes the data to ROS topic
        """
        
        depth_measurement = Measurement()
        pressure_msg = Pressure()
        
        # Populate the pressure message (only used for telemetry)
        pressure_msg.header.frame_id = 'bluerov_depth'
        pressure_msg.header.stamp = rospy.Time.now()
        pressure_msg.temperature = data['temperature'] / 100.0 # (in ºC)
        pressure_msg.pressure = data['press_abs'] # (in mBar)

        # Publish the pressure message
        self.pressure_pub.publish(pressure_msg)

        # Populate the depth message (used by the navigation filter)
        depth_measurement.header.frame_id = 'bluerov_depth'
        depth_measurement.header.stamp = rospy.Time.now()

        # Compute the depth based on the read pressure levels 
        depth_measurement.value = [data['press_abs'] * 9.90748885684893 / 1000.0 - self.depth_atm_pressure]
        depth_measurement.noise = [0.1]

        # Publish the depth message to the filter topic and the drivers topic
        self.depth_pub.publish(depth_measurement)
        self.depth_pub_data.publish(depth_measurement)
        

    def gps_to_ros(self, raw_data, position_data):
        """
        Method that receives a mavlink message of GPS_RAW_INT and GLOBAL_POSITION_INT in dictionary format
        and publishes the data to ROS topic
        """

        gps_msg = NavSatFix()
        gps_msg.header.frame_id = 'bluerov_gnss'
        gps_msg.header.stamp = rospy.Time.now()
        
        # Check if the GPS message will be good or not
        if raw_data['satellites_visible'] < 4:
            gps_msg.status.status = -1
        else:
            gps_msg.status.status = gps_msg.status.STATUS_FIX
        
        gps_msg.status.service = gps_msg.status.SERVICE_GPS

        # Update with the latest data from lat-long and altitude
        gps_msg.latitude = position_data['lat'] * 1.0e-7
        gps_msg.longitude = position_data['lon'] * 1.0e-7
        gps_msg.altitude = position_data['alt']

        self.gps_pub.publish(gps_msg)


def main():
    """
    Main - the entry point of the program
    """

    # Start the ROS node
    rospy.init_node('bluerov_interface_node')

    # Connection to the pixhawk and setup the number of thrusters in use
    udp_mavlink: str = rospy.get_param('~udp_mavlink', default='udpin:192.168.2.1:14550')
    num_thrusters: int = rospy.get_param('~num_thrusters', default=8)

    # Initiate the autopilot class (with 1 extra-thruster - control the lights through the thruster driver!)
    rospy.loginfo(udp_mavlink)
    rospy.loginfo(num_thrusters+1)

    # Create an autopilot to control the thrusters (not ideal, but if it works, it works)
    # Note lights are controlled like thrusters, hence num_thrusters+1
    with ExtendedAutopilot(udp_mavlink, client=True, thrusters=num_thrusters+1) as autopilot, autopilot.per_thruster_control():

        # Create an interface node
        bluerov_interface = BluerovInterfaceNode(autopilot, num_thrusters)

        # Go into spin and let the callbacks do all the work
        rospy.spin()
        
        # Make sure we set the lights value to zero (1100 PWM)
        bluerov_interface.autopilot.set_servo(bluerov_interface.num_thrusters + 1, 1100 + (0.0*100))

        # Make sure we set the thrusters value to zero (1500 PWM)
        for i in range(0, bluerov_interface.num_thrusters):
            bluerov_interface.autopilot.set_servo(i + 1, 1500.0)


if __name__ == '__main__':
    main()