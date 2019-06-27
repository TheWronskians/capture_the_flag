import numpy as np
import matplotlib.pyplot as plt
import cv2

import colour
from colour import RED, WHITE, ORANGE, YELLOW, GREEN, BLUE
import prm
import motion
import client

if __name__ == "__main__":

    N = 100 #Number of samples for PRM
    k = 10  #Number of nearest neighbours for PRM
    W = 640 #Width of arena
    H = 480 #Height of arena
    Pad = 80 #Padding around edges
    graph = prm.initGraph(N,W,H,Pad,k)
    cap = cv2.VideoCapture(0)
    client.createConnections()	#Creates three connections to servers
    #PID Stuff
    last = 0
    lastl = 0
    k_angle = [0.15,0.005,0]
    k_linear = [0.001,0.00005,0]
    I = 0
    Il = 0
    angle = 0
    distance = 0
    tol = 0.01
    toll = 0.01
    turnVel = 3
    velocity = 3

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
        back = cs[WHITE]
        front = cs[RED]
        goal = cs[GREEN]

        if np.isnan(goal[0]) or np.isnan(goal[1]):
            # Can't see ball
            goal = [W/2,H/2]

        # Set nodes
        startNode = prm.Node(back[0],back[1],N)
        goalNode = prm.Node(goal[0],goal[1],N+1)
        frontNode = prm.Node(front[0],front[1],N)
        enemy = prm.Obstacle(-10,-10,1)
        ball = prm.Obstacle(-10,-10,1)

        #Find path
        x,y = prm.pathPlan(graph,startNode,goalNode,enemy,ball,k,avoidBall=False,draw=False)
        next = prm.Node(x,y,0)
        last = angle
        lastl = distance

        if iteration % correction_step == 0:
            #Reset points to correct any mistakes
            turnVel = 3
            velocity = 3


        if(turnVel!=0):
            #Turn to goal
            turnVel,angle = motion.PID_Angle(startNode,frontNode,next,k_angle,I,last,tol)
            limitSet = [0, 0, 0, 0, 0, turnVel]
        elif velocity!=0:
            #Move to goal
            velocity,distance = motion.PID_Linear(startNode,next,k_linear,Il,lastl,toll)
            limitSet = [velocity, 0, 0, 0, 0, 0]
        else:
            #Do nothing
            limitSet = [0, 0, 0, 0, 0, 0]

        I += angle
        Il += distance

        client.sendToServer(limitSet) #Sending sets to servers

        results, times = client.getReplies()


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
    client.closeConnections() #Closing all connections to servers.

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()
