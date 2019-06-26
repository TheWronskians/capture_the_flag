import numpy as np
import matplotlib.pyplot as plt
import cv2

import colour
import prm
import motion
import client

if __name__ == "__main__":

    N = 100
    k = 10
    graph = prm.initGraph(N,640,480,80,k)
    cap = cv2.VideoCapture(0)
    client.createConnections()	#Creates three connections to servers
    last = 0
    lastl = 0
    I = 0
    kp = 0.15
    ki = 0.005
    kd = 0
    Il = 0
    kpl = 0.001
    kil = 0.00005
    kdl = 0
    angle = 0
    distance = 0
    tol = 0.01
    toll = 0.01
    turnVel = 3
    velocity = 3
    while(True):
        # Capture frame-by-frame
        ret, frame = cap.read()
        #frame = cv2.cvtColor(frame, cv2.COLOR_HSV2RGB)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        image = np.array(frame).astype(np.float)/255
        cs = colour.findCenters(image,tol=0.1)

        '''
        fig, ax = plt.subplots()
        ax.imshow(image)
        for c in cs:
            ax.scatter(c[1], c[0], s=50, color='cyan')
        plt.show()
        '''
        back = cs[1]
        front = cs[0]
        goal = cs[4]
        startNode = prm.Node(back[0],back[1],N)
        goalNode = prm.Node(goal[0],goal[1],N+1)
        frontNode = prm.Node(front[0],front[1],N)
        enemy = prm.Obstacle(-10,-10,1)
        ball = prm.Obstacle(-10,-10,1)
        x,y = prm.pathPlan(graph,startNode,goalNode,enemy,ball,k,avoidBall=False,draw=False)
        next = prm.Node(x,y,0)
        last = angle
        lastl = distance
        if(turnVel!=0):
            turnVel,angle = motion.PID_Angle(startNode,frontNode,next,kp,ki,kd,I,last,tol)
            limitSet = [0, 0, 0, 0, 0, turnVel]
        elif velocity!=0:
            velocity,distance = motion.PID_Linear(startNode,next,kpl,kil,kdl,Il,lastl,toll)
            limitSet = [velocity, 0, 0, 0, 0, 0]
        else:
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
    client.closeConnections() #Closing all connections to servers.

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()
