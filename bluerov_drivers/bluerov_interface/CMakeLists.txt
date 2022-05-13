cmake_minimum_required(VERSION 3.3)
project(bluerov_interface)

find_package(catkin REQUIRED
 COMPONENTS
 std_msgs
 sensor_msgs
 medusa_msgs
 dsor_msgs
 rospy
)

catkin_python_setup()

catkin_package(CATKIN_DEPENDS)
