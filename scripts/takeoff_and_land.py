#!/usr/bin/env python


"""
Joshua Bruton
Kimessha Paupamah
"""

import rospy
import time
from geometry_msgs.msg import Twist, Vector3 # for sending commands to the drone
from std_msgs.msg import Empty # for land/takeoff/emergency
from ardrone_autonomy.msg import Navdata # for receiving navdata feedback

def takeoff_and_land():
	rospy.init_node('mover', anonymous=True)
	to = rospy.Publisher('/ardrone/takeoff', Empty, queue_size=10)
	la = rospy.Publisher('/ardrone/land', Empty, queue_size=10)		
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():		
		to.publish()
		time.sleep(5)
		la.publish()
	'''
	while not rospy.is_shutdown():
		pub.publish(Twist(Vector3(0.3,0,0), Vector3(0,0,0)))
		time.sleep(5)
		pub.publish(Twist(Vector3(0,0,0), Vector3(0,0,0.1)))
		time.sleep(1)
		pub.publish(Twist(Vector3(0.2,0,0), Vector3(0,0,0.1)))
		rate.sleep()
	'''
takeoff_and_land()
