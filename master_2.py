import numpy as np
import matplotlib.pyplot as plt
import cv2

import colour
from colour import RED, WHITE, ORANGE, YELLOW, GREEN, BLUE
import prm
import motion
import client

# server names for turtlebots which we connect to
servers = ["10.199.61.21", ""]
# ports in which the messages pass through
ports = [12007, 12006]
# sockets for each port/server connection
sockets = [0, 1]

if __name__ == "__main__":

    N = 100 # Number of samples for PRM
    k = 10  # Number of nearest neighbours for PRM
    W = 640 # Width of arena
    H = 480 # Height of arena
    Pad = 80 # Padding around edges
    graph = prm.initGraph(N,W,H,Pad,k)
    cap = cv2.VideoCapture(0)
    # turtlebot server 1
    client.create_connections(servers[0], ports[0], sockets[0])
    # turtlebot server 2
    # client.create_connections(servers[1], ports[1], sockets[1])

    # PID Stuff
    prev_angle = 0
    prev_distance = 0
    # kp, ki, kd
    k_angle = [0.15, 0.005, 0]
    k_distance = [0.001, 0.00005, 0]

    # integral control
    K_i_angle = 0
    K_i_distance = 0
    #
    angle = 0
    distance = 0
    # tolerance before we are satisfied
    tol_angle = 0.01
    tol_distance = 0.01

    # setting velocities to a non-zero number
    angular_velocity = 3
    velocity = 3

    # refresh rate
    iteration = 0
    correction_step = 20

    while(True):
        ret, frame = cap.read() # Capture frame-by-frame
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) # Convert from BGR to RGB
        image = np.array(frame).astype(np.float)/255 # Convert from Int8 to float
        cs = colour.findCenters(image,tol=0.1) # Find colour centres

        '''
        fig, ax = plt.subplots()
        ax.imshow(image)
        for c in cs:
            ax.scatter(c[1], c[0], s=50, color='cyan')
        plt.show()
        '''
        # Define points
        # turtlebot
        back = cs[WHITE]
        front = cs[RED]
        goal = cs[GREEN]

        if np.isnan(goal[0]) or np.isnan(goal[1]):
            # Can't see ball
            goal = [W/2, H/2]

        # Set nodes
        startNode = prm.Node(back[0], back[1], N)
        goalNode = prm.Node(goal[0], goal[1], N + 1)
        frontNode = prm.Node(front[0], front[1], N)
        # obstacle
        enemy = prm.Obstacle(-10, -10, 1)
        # first goal
        ball = prm.Obstacle(-10, -10, 1)

        #Find path
        x, y = prm.pathPlan(graph, startNode, goalNode, enemy, ball, k, avoidBall=False, draw=False)
        next = prm.Node(x, y, 0)
        prev_angle = angle
        prev_distance = distance

        if iteration % correction_step == 0:
            #Reset points to correct any mistakes
            turnVel = 3
            velocity = 3
        if(turnVel!=0):
            #Turn to goal
            # angular_velocity, angle = motion.PID_Angle(startNode, frontNode, next, k_angle, K_i_angle, prev_angle, tol_angle)
            angular_velocity = motion.Stochastic_Angle(startNode, frontNode, next, tol_angle)
            limitSet = [0, 0, 0, 0, 0, angular_velocity]
        elif velocity!=0:
            #Move to goal
            # velocity, distance = motion.PID_Linear(startNode, next, k_distance, K_i_distance, prev_distance, tol_distance)
            velocity = motion.Stochastic_Linear(startNode, next, tol_distance)
            limitSet = [velocity, 0, 0, 0, 0, 0]
        else:
            #Do nothing
            limitSet = [0, 0, 0, 0, 0, 0]

        # summing the errors
        # K_i_angle += angle
        # K_i_distance += distance

        # sending to turtlebot one
        client.send_to_server(limitSet, servers[0], sockets[0])
        # sending to turtlebot two
        # client.send_to_server(limitSet, servers[1], sockets[1])
        # reponse from turtlebot one
        results = client.get_replies(sockets[0])
        # response from turtlebot two
        # results = client.get_replies(sockets[1])



        '''
        fig, ax = plt.subplots()
        ax.imshow(image)
        for c in cs:
            ax.scatter(c[1], c[0], s=50, color='cyan')
        ax.scatter(y,x,s=50,color='purple')
        plt.show()
        '''
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        iteration += 1
    client.close_connections(sockets[0]) #Closing all connections to servers.
    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()
