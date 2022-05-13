#!/usr/bin/env python

"""
@author: Marcelo Fialho Jacinto
@email: marcelo.jacinto@tecnico.ulisboa.pt
@date: 25/04/2022
@licence: MIT
"""
import rospy
import pygame
import time
from std_msgs.msg import Float64
from auv_msgs.msg import NavigationStatus
from joystick_controller import ControlAssignment
from dsor_msgs.srv import DoubleValue


class RemoteControllerNode:
    """
    Remote controller ROS node class. Receives from a joystick (using pygame) the desired controls for
    the inner-loops and publishes these periodically (at a pre-defined frequency) to the inner-loops of the
    vehicle
    """

    def __init__(self):
        """
        Class constructor. Initializes the ros node, loads the parameters from the ros parameter server, creates the inner-loops
        publishers and initializes the timer that publishes the desired inputs to the inner-loops
        """

        # ---Initialize the ROS NODE---
        rospy.init_node('remote_controller_node')
        self.node_frequency = rospy.get_param('~node_frequency', 10)
        self.h_timerActivate = False

        # ---Load the ROS configurations---
        self.initializePublishers()

        # --- Define the last gimbal angle
        self._last_gimbal_angle = 0.0

        # --- Define the last lights state
        self._last_lights_state = 0.0

        # --- Define the last speed gain
        self._last_speed_gain = 0.5

        # ---Initialize the pygame to read the inputs from the joystick
        self.initializeJoystick()

        # ---Start the ROS NODE callback---
        self.initializeTimer()

    def timerCallback(self, event):
        """
        Callback used to publish to the inner-loops the desired control inputs
        :param event: A timer event - unused but required by the ROS API
        """

         # Check for the desired inputs for the inner-loops
        desired_inputs = self.control_assignment.check_events()

        # Increment the lights current value according to the joytsick successive button presses
        desired_inputs['lights'] = self.setLightsValue(desired_inputs['lights'])

        # Increment/Decrement the speed gain to apply to surge, sway and heave according to the joystick successive button presses
        desired_inputs['speed_gains'] = self.setSpeedGain(desired_inputs['speed_gains'])

        # Apply the speed gain to the desired inputs in surge, sway and heave
        desired_inputs['surge'] = desired_inputs['surge'] * desired_inputs['speed_gains']
        desired_inputs['sway'] = desired_inputs['sway'] * desired_inputs['speed_gains']
        desired_inputs['heave'] = desired_inputs['heave'] * desired_inputs['speed_gains']
        desired_inputs['yaw_rate'] = desired_inputs['yaw_rate'] * desired_inputs['speed_gains']

        # Log the desired inputs
        print(desired_inputs)

        # Publish surge, sway and heave
        self.surge_pub.publish(desired_inputs["surge"])
        self.sway_pub.publish(desired_inputs["sway"])
        self.heave_pub.publish(desired_inputs["heave"])
    
        # Publish yaw-rate
        self.yaw_rate_pub.publish(desired_inputs["yaw_rate"])

        # Publish the gimbal angle
        self.gimbal_pub.publish(desired_inputs["camera_angle"])

        # Call the services to set the lights value
        self.set_lights(desired_inputs["lights"])

    def initializeTimer(self):
        """
        Method that starts the system timer that periodically calls a callback
        """
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timerCallback)


    def initializePublishers(self):
        """
        Method that initializes the ROS publishers (to publish the references for the inner-loops)
        """
        rospy.loginfo('Initializing Publishers for RemoteControllerNode')

        # Defining the publishers for linear speed references
        self.surge_pub = rospy.Publisher(rospy.get_param('~topics/publishers/surge'), Float64, queue_size=1)
        self.sway_pub = rospy.Publisher(rospy.get_param('~topics/publishers/sway'), Float64, queue_size=1)
        self.heave_pub = rospy.Publisher(rospy.get_param('~topics/publishers/heave'), Float64, queue_size=1)

        # Publish the angular speed references
        self.yaw_rate_pub = rospy.Publisher(rospy.get_param('~topics/publishers/yaw_rate'), Float64, queue_size=1)

        # Publish gimbal references
        self.gimbal_pub = rospy.Publisher(rospy.get_param('~topics/publishers/gimbal'), Float64, queue_size=1)

    def set_lights(self, value: float):

        try:
            double_value = rospy.ServiceProxy(rospy.get_param('~topics/services/lights'), DoubleValue)
            resp = double_value(value)
            return resp.res
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def initializeJoystick(self):
        """
        Method that initializes the joystick driver (using pygame)
        """

        self.control_assignment = None

        # Retry to enter the remote control if some error is detected in the connection
        while self.control_assignment is None:

            # Create the joystick object
            try:
                # Get the controller configurations from the parameter server and save them in a dictionary
                controller_configurations = {"surge": rospy.get_param('~button_assignment/surge'),
                                            "sway": rospy.get_param('~button_assignment/sway'),
                                            "heave": rospy.get_param('~button_assignment/heave'),
                                            "yaw_rate": rospy.get_param('~button_assignment/yaw_rate'),
                                            "camera_angle": rospy.get_param('~button_assignment/camera_angle'),
                                            "lights": rospy.get_param('~button_assignment/lights'),
                                            "speed_gains": rospy.get_param('~button_assignment/speed_gains')}

                # Initialize the main joystick and bind key assignments according to configuration in ros parameter server
                self.control_assignment = ControlAssignment(controller_configurations)

            except Exception as e:
                # Check if we are able to create the joystick object. If not, terminate the node and send an error message
                print("Could not start joystick driver: " + str(e))
                print("Will retry in 2 seconds")
                time.sleep(2)

    def setLightsValue(self, increment_value):
        """
        Method used to increment the current lights value by some value
        Returns the final desired lights value 
        """

        self._last_lights_state += increment_value

        # Saturate the angle between -45 and 45 degrees
        if self._last_lights_state > 8:
            self._last_lights_state = 8
        elif self._last_lights_state < 0:
            self._last_lights_state = 0

        return self._last_lights_state

    def setSpeedGain(self, increment_value):

        self._last_speed_gain += increment_value

        # Saturate the speed gain between 0.25 and 1.0
        if self._last_speed_gain > 1.0:
            self._last_speed_gain = 1.0
        elif self._last_speed_gain < 0.25:
            self._last_speed_gain = 0.25

        return self._last_speed_gain


def main():
    """
    Initialize the RemoteControllerNode and let the timer callback do all the work
    """
    remote_controller = RemoteControllerNode()
    rospy.spin()


if __name__ == '__main__':
    main()
