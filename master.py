import numpy as np
import matplotlib.pyplot as plt
import cv2
import time

import colour
from colour import RED, WHITE, ORANGE, YELLOW, GREEN, BLUE
import prm
import motion
import client

if __name__ == "__main__":

    N = 9 #Number of samples for PRM
    k = 10  #Number of nearest neighbours for PRM
    W = 640 #Width of arena
    H = 480 #Height of arena
    Pad = 50 #Padding around edges
    manual = True
    graph = prm.initGraph(N,H,W,Pad,k,manual)
    cap = cv2.VideoCapture(0)
    client.createConnections()	#Creates three connections to servers
    #PID Stuff

    k_angle = [0.15,0.025,0.1]
    k_linear = [0.002,0.0001,0.005]
    I = 0
    Il = 0
    last = 0
    lastl = 0
    angle = 0
    distance = 0
    tol = 0.25
    toll = 0.01
    turnVel = 3
    velocity = 3

    enemyRadius = 60
    ballRadius = 10

    returnVel = 0.3

    iteration = 0
    correction_step = 20

    lastBall = [H/2,W/2]

    while(True):

        ret, frame = cap.read() # Capture frame-by-frame
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) # Convert from BGR to RGB
        image = np.array(frame_rgb).astype(np.float)/255 # Convert from Int8 to float
        cs = colour.findCenters(image,tol=0.1,draw=True,frame=frame) # Find colour centres
        print(image.shape)

        '''
        fig, ax = plt.subplots()
        ax.imshow(image)
        for c in cs:
            ax.scatter(c[1], c[0], s=50, color='cyan')
        plt.show()
        #'''
        # Define points
        back = cs[WHITE]
        front = cs[RED]
        start = [(back[0]+front[0])/2,(back[1]+front[1])/2]
        goal = cs[GREEN]
        ball = cs[GREEN]
        enemy_front = cs[ORANGE]
        enemy_back = cs[YELLOW]
        enemy = [(enemy_back[0]+enemy_front[0])/2,(enemy_back[1]+enemy_front[1])/2]

        if np.isnan(enemy[0]):
            enemy[0] = -enemyRadius
        if np.isnan(enemy[1]):
            enemy[1] = -enemyRadius

        if np.isnan(ball[0]) or np.isnan(ball[1]):
            ball = lastBall

        if np.isnan(goal[0]) or np.isnan(goal[1]):
            # Can't see ball
            print("Can't see ball, going to centre")
            goal = lastBall
        else:
            lastBall = goal


        if not(np.isnan(start[0]) or np.isnan(start[1])):
            goalDisp = [0,0]
            startNode = prm.Node(start[0],start[1],N)
            goalNode = prm.Node(goal[0]+goalDisp[0],goal[1]+goalDisp[1],N+1)
            frontNode = prm.Node(front[0],front[1],N)
            enemy = prm.Obstacle(enemy[0],enemy[1],enemyRadius)
            ballNode = prm.Obstacle(ball[0],ball[1],ballRadius)

            #Find path
            x,y = prm.pathPlan(graph,startNode,goalNode,enemy,ballNode,k,avoidBall=False,draw=True,w=640,h=480,frame=frame)
            next = prm.Node(x,y,0)

            if motion.dist(startNode,goalNode)<100:
                print("RESET NEXT TO GOAL")
                next = prm.Node(goalNode.x,goalNode.y,0)

            last = angle
            lastl = distance

            if(turnVel!=0):
                #Turn to goal
                Il = 0
                turnVel,angle = motion.PID_Angle(startNode,frontNode,next,k_angle,I,last,tol)
                if np.isnan(turnVel):
                    print("ANGLE IS NAN")
                    turnVel = 0
                limitSet = [0, 0, 0, 0, 0, turnVel]
                client.sendToServer(limitSet) #Sending sets to servers
                results, times = client.getReplies()
            elif velocity!=0:
                #Move to goal
                I = 0
                velocity,distance = motion.PID_Linear(startNode,next,k_linear,Il,lastl,toll)
                if np.isnan(velocity):
                    print("VELOCITY IS NAN")
                    velocity = 0
                limitSet = [velocity, 0, 0, 0, 0, 0]
                client.sendToServer(limitSet) #Sending sets to servers
                results, times = client.getReplies()
            else:
                #Do nothing
                turnVel = 3
                velocity = 3
                print("turnVel = %i, Velocity = %i" %(turnVel,velocity))
                #limitSet = [0, 0, 0, 0, 0, 0]

            I += angle
            Il += distance

        cv2.imshow('frame',frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        iteration += 1
        print("\n")

    client.closeConnections() #Closing all connections to servers.

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()
