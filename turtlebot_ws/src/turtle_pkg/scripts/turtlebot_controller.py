#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import time
from socket import *
import pickle #Pickle used for data transfering.
from primeCalc import *
import time

def move(x, y, z, ax, ay, az):
    # Starts a new node
    rospy.init_node('turtlebot_node', anonymous=True)
    velocity_publisher = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
    velocity_msg = Twist()

    print("Moving turtlebot")

    velocity_msg.linear.x = 0.3

    while not rospy.is_shutdown():
        # moving forward for at 0.3 speed for 5 seconds
        velocity_msg.linear.x = 0.3
        time_end = time.time() + 5
        while(time.time() < time_end):
            velocity_publisher.publish(velocity_msg)

        velocity_msg.linear.x = 0
        velocity_publisher.publish(velocity_msg)

        # rotating at 0.1 for 1 second
        velocity_msg.angular.x = 0.1
        time_end = time.time() + 1
        while(time.time() < time_end):
            velocity_publisher.publish(velocity_msg)

        velocity_msg.angular.x = 0
        velocity_publisher.publish(velocity_msg)

        # moving forware again at 0.2 speed for 5 seconds
        velocity_msg.linear.x = 0.2
        time_end = time.time() + 5
        while(time.time() < time_end):
            velocity_publisher.publish(velocity_msg)

        velocity_msg.linear.x = 0
        velocity_publisher.publish(velocity_msg)

        # exiting the loop
        break
if __name__ == '__main__':
    #Set server socket settings.
    serverPort = 12007
    serverSocket = socket(AF_INET, SOCK_STREAM)
    serverSocket.bind(("",serverPort))
    serverSocket.listen(1)
    print("The server is ready to receive")
    while 1:
    	connectionSocket, addr = serverSocket.accept() #connecting to
    	while True:
    		pickledLimits = connectionSocket.recv(1024)#Recieve data in Pickle form.
    		limits = pickle.loads(pickledLimits)#Convert data to array
    		num , t = countPrimes(limits[0],limits[1])
            # getting the x, y, z, ax, ay, az from the camera
            # x, y, z, ax, ay, az = limits
            # try:
                # sending these limits to the move function
                # move(x, y, z, ax, ay, az)
            # except rospy.ROSInterruptException: pass
    		toSend = [num, t] #Preparing data to be sent
    		connectionSocket.send(pickle.dumps(toSend)) #Converting to Pickle form and sending
    	connectionSocket.close()#Closing connections.
