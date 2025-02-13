#!/bin/python

import rospy
from matplotlib import pyplot as plt
from dsor_msgs.msg import Measurement
import datetime
def callback_fn(msg):
	if msg.header.frame_id =="bluerov_heavy0/altimeter2_link": 
		print("here1")
		x.publish(msg)
	
	if msg.header.frame_id == "bluerov_heavy0/altimeter1_link":
		print("here2")
		y.publish(msg)	

rospy.init_node("Altitude_Plotter")

rospy.Subscriber("/bluerov_heavy0/measurement/position",Measurement,callback_fn)
x = rospy.Publisher("/bluerov_heavy0/altimeter2",Measurement,queue_size=5)
y = rospy.Publisher("/bluerov_heavy0/altimeter1",Measurement,queue_size=5)
rospy.spin()
