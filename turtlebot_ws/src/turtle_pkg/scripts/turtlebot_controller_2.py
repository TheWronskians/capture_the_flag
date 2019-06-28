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
    # velocity_msg.linear.x = 0.3
    # velocity_msg.linear.x = x


    while not rospy.is_shutdown():
        time_end = time.time() + 1
        while(time.time() < time_end):
            velocity_msg.linear.x = x;
            velocity_msg.linear.y = y;
            velocity_msg.linear.z = z;
            velocity_msg.angular.x = ax;
            velocity_msg.angular.y = ay;
            velocity_msg.angular.z = az;
            velocity_publisher.publish(velocity_msg)
        # exiting the loop
        break
if __name__ == '__main__':
    #Set server socket settings.
    serverPort = 12006
    serverSocket = socket(AF_INET, SOCK_STREAM)
    serverSocket.bind(("",serverPort))
    serverSocket.listen(1)
    print("The server is ready to receive")
    while 1:
        connectionSocket, addr = serverSocket.accept() #connecting to
        while True:
            pickledLimits = connectionSocket.recv(1024)#Recieve data in Pickle form.
            limits = pickle.loads(pickledLimits)
            x, y, z, ax, ay, az = limits
            try:
                move(x, y, z, ax, ay, az)
                toSend = [1, 21]
                connectionSocket.send(pickle.dumps(toSend))
            except rospy.ROSInterruptException: pass
            # num , t = countPrimes(limits[0],limits[1])
            # toSend = [1, 21] #Preparing data to be sent
            # connectionSocket.send(pickle.dumps(toSend)) #Converting to Pickle form and sending
        connectionSocket.close()#Closing connections.




###########################################################################################
# moving forward for at 0.3 speed for 5 seconds
# velocity_msg.linear.x = 0.3
# velocity_msg.linear.x = x
# time_end = time.time() + 5
# while(time.time() < time_end):
#     velocity_publisher.publish(velocity_msg)
#
# velocity_msg.linear.x = 0
# velocity_publisher.publish(velocity_msg)
#
# # rotating at 0.1 for 1 second
# # velocity_msg.angular.x = 0.1
# velocity_msg.angular.z = ax
# time_end = time.time() + 5
# while(time.time() < time_end):
#     velocity_publisher.publish(velocity_msg)
#
# velocity_msg.angular.z = 0
# velocity_publisher.publish(velocity_msg)
#
# # moving forware again at 0.2 speed for 5 seconds
# # velocity_msg.linear.x = 0.2
# velocity_msg.linear.x = x - 0.1
# time_end = time.time() + 5
# while(time.time() < time_end):
#     velocity_publisher.publish(velocity_msg)
#
# velocity_msg.linear.x = 0
# velocity_publisher.publish(velocity_msg)
