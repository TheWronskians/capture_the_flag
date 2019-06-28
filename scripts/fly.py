#!/usr/bin/env python


"""
Joshua Bruton
Kimessha Paupamah
"""

import rospy
import time
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import Twist # for sending commands to the drone
from std_msgs.msg import Empty # for land/takeoff/emergency
from ardrone_autonomy.msg import Navdata # for receiving navdata feedback

def land():
	la = rospy.Publisher('/ardrone/land', Empty, queue_size=10)
	la.publish()

def flyTo():
	rospy.init_node('mover', anonymous=True)
	to = rospy.Publisher('/ardrone/takeoff', Empty, queue_size=10)
	pub_move = rospy.Publisher('/cmd_vel', Twist, queue_size=10)		
	rate = rospy.Rate(10)

	for i in range(0,30):
		to.publish()
		rate.sleep()

	coords=[0,0,0]
	coords[0] = raw_input("Input x:")
	coords[1] = raw_input("Input y:")
	coords[2] = raw_input("Input z:")

	while not rospy.is_shutdown():
		world = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
		drone = world("quadrotor", "")
		x_pos = drone.pose.position.x
		y_pos = drone.pose.position.y
		z_pos = drone.pose.position.z
		new = Twist()
		new.linear.x=(float(coords[0])-x_pos)/4.0
		new.linear.y=(float(coords[1])-y_pos)/4.0
		new.linear.z=(float(coords[2])-z_pos)/4.0		
		pub_move.publish(new)
		rospy.on_shutdown(land)

	'''
	while not rospy.is_shutdown():
		pub.publish(Twist(Vector3(0.3,0,0), Vector3(0,0,0)))
		time.sleep(5)
		pub.publish(Twist(Vector3(0,0,0), Vector3(0,0,0.1)))
		time.sleep(1)
		pub.publish(Twist(Vector3(0.2,0,0), Vector3(0,0,0.1)))
		rate.sleep()
	'''
flyTo()
