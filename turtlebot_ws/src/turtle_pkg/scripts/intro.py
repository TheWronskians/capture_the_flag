#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import time

def move():
    # Starts a new node
    rospy.init_node('turtlebot_node', anonymous=True)
    velocity_publisher = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
    velocity_msg = Twist()

    print("Moving turtlebot")

    velocity_msg.linear.x = 0.3

    while not rospy.is_shutdown():
        # moving forward for at 0.3 speed for 5 seconds
        velocity_msg.linear.x = 0.5
        time_end = time.time() + 10
        while(time.time() < time_end):
            velocity_publisher.publish(velocity_msg)

        velocity_msg.linear.x = 0
        velocity_publisher.publish(velocity_msg)

        # rotating at 0.1 for 1 second
        #velocity_msg.angular.x = 0.1
        #time_end = time.time() + 1
        #while(time.time() < time_end):
        #    velocity_publisher.publish(velocity_msg)

        #velocity_msg.angular.x = 0
        #velocity_publisher.publish(velocity_msg)

        # moving forware again at 0.2 speed for 5 seconds
        #velocity_msg.linear.x = 0.2
        #time_end = time.time() + 5
        #while(time.time() < time_end):
        #    velocity_publisher.publish(velocity_msg)

        #velocity_msg.linear.x = 0
        #velocity_publisher.publish(velocity_msg)

        # exiting the loop
        break
if __name__ == '__main__':
    try:
        move()
    except rospy.ROSInterruptException: pass
