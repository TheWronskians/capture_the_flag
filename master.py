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
    graph = prm.initGraph(N,640,480,40,k)
    cap = cv2.VideoCapture(0)
    client.createConnections()	#Creates three connections to servers
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
        enemy = prm.Obstacle(-10,-10,1)
        ball = prm.Obstacle(-10,-10,1)
        x,y = prm.pathPlan(graph,startNode,goalNode,enemy,ball,k,avoidBall=False,draw=False)
        next = prm.Node(x,y,0)
        kp = 0.5
        turnVel = motion.turnToGoal(back,front,next,kp=0.5,tol=0.5)
        limitSet = [0, 0, 0, 0, 0, turnVel]
        sendToServer(limitSet) #Sending sets to servers

        results, times = getReplies()


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
    closeConnections() #Closing all connections to servers.

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()
