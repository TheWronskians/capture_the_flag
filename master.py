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

    N = 100 #Number of samples for PRM
    k = 10  #Number of nearest neighbours for PRM
    W = 640 #Width of arena
    H = 480 #Height of arena
    Pad = 50 #Padding around edges
    graph = prm.initGraph(N,H,W,Pad,k)
    cap = cv2.VideoCapture(0)
    client.createConnections()	#Creates three connections to servers
    #PID Stuff

    k_angle = [0.15,0.01,0]
    k_linear = [0.001,0,0]
    I = 0
    Il = 0
    last = 0
    lastl = 0
    angle = 0
    distance = 0
    tol = 0.01
    toll = 0.01
    turnVel = 3
    velocity = 3

    returnVel = 0.3

    iteration = 0
    correction_step = 100

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
        #'''
        # Define points
        back = cs[WHITE]
        front = cs[RED]
        start = [(back[0]+front[0])/2,(back[1]+front[1])/2]
        goal = cs[GREEN]
        enemy = cs[ORANGE]


        resetPID = False


        if np.isnan(goal[0]) or np.isnan(goal[1]):
            # Can't see ball
            print("Can't see ball, going to centre")
            goal = [H/2,W/2]

        noBack = False
        noFront = False
        if np.isnan(back[0]) or np.isnan(back[1]):
            noBack = True
        if np.isnan(front[0]) or np.isnan(front[1]):
            noFront = True

        if not noFront and noBack:
            #Can see front, but not back
            limitSet = [returnVel,0,0,0,0,0]
            print("ONLY SEE FRONT")
            resetPID = True

        if not noBack and noFront:
            #Can see back but not front
            limitSet = [-returnVel,0,0,0,0,0]
            print("ONLY SEE BACK")
            resetPID = True

        if noBack and noFront:
            #Can't see either
            limitSet = [0,0,0,0,0,0.3]
            print("SPIN MODE")
            resetPID = True

        if iteration % correction_step == 0:
            resetPID = True

        if resetPID:
            turnVel = 3
            velocity = 3
            I = 0
            Il = 0
            last = 0
            lastl = 0
            angle = 0
            distance = 0

        if (not noBack) and (not noFront):
            # Set nodes
            goalDisp = [0,0]
            startNode = prm.Node(start[0],start[1],N)
            goalNode = prm.Node(goal[0]+goalDisp[0],goal[1]+goalDisp[1],N+1)
            frontNode = prm.Node(front[0],front[1],N)
            enemy = prm.Obstacle(enemy[0],enemy[1],60)
            ball = prm.Obstacle(cs[GREEN][0],cs[GREEN][1],30)

            #Find path
            x,y = prm.pathPlan(graph,startNode,goalNode,enemy,ball,k,avoidBall=False,draw=False,w=640,h=480)
            next = prm.Node(x,y,0)
            last = angle
            lastl = distance


            if(turnVel!=0):
                #Turn to goal
                turnVel,angle = motion.PID_Angle(startNode,frontNode,next,k_angle,I,last,tol)
                if np.isnan(turnVel):
                    print("SPIN MODE 2")
                    turnVel = -0.3
                limitSet = [0, 0, 0, 0, 0, turnVel]
            elif velocity!=0:
                #Move to goal
                velocity,distance = motion.PID_Linear(startNode,next,k_linear,Il,lastl,toll)
                if np.isnan(velocity):
                    print("REVERSE!")
                    velocity = -0.3
                limitSet = [velocity, 0, 0, 0, 0, 0]
            else:
                #Do nothing
                limitSet = [0, 0, 0, 0, 0, 0]

            I += angle
            Il += distance

        if resetPID:
            endtime = time.time()+3
            while time.time()<endtime:
                print("NEED TO CORRECT")
                client.sendToServer(limitSet) #Sending sets to servers
                results, times = client.getReplies()

        else:
            client.sendToServer(limitSet) #Sending sets to servers

            results, times = client.getReplies()

        #print("Can see ball: %s. Goal = [%i,%i], Front = [%i,%i], Next = [%i,%i]" %(ballSee,goalNode.x,goalNode.y,frontNode.x,frontNode.y,next.x, next.y))
        print("Next: ",next.x, next.y)


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
        print("\n")
        '''
        iterWait = time.time()+0.1
        while time.time()<iterWait:
            pass
        #'''
    client.closeConnections() #Closing all connections to servers.

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()
