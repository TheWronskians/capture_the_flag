import numpy as np
import matplotlib.pyplot as plt
import cv2
import time

import colour
from colour import RED, WHITE, BLUE, YELLOW, GREEN
import prm
import motion
import client_2 as client

# server names for turtlebots which we connect to
servers = ["10.199.61.21", "10.199.26.14"]
# ports in which the messages pass through
ports = [12007, 12006]
# sockets for each port/server connection
sockets = [0, 1]
# 0 = Red-White
# 1 = Orange-Yellow

N = 9 #Number of samples for PRM
k = 10  #Number of nearest neighbours for PRM
W = 640 #Width of arena
H = 480 #Height of arena
Pad = 50 #Padding around edges
manual = True

lastBall = [[H/2,W/2],[H/2,W/2]]

#PID Stuff

k_angle = [[0.15,0.025,0.1],[0.15,0.025,0.1]]
k_linear = [[0.002,0.0001,0.005],[0.002,0.0001,0.005]]
I = [0,0]
Il = [0,0]
last = [0,0]
lastl = [0,0]
angle = [0,0]
distance = [0,0]
tol = 0.25
toll = 0.01
turnVel = [3,3]
velocity = [3,3]

enemyRadius = 80
ballRadius = 20



def run(bot):
    ball = cs[GREEN]

    if bot==0:
        # Define points
        back = cs[WHITE]
        front = cs[RED]
        start = [(back[0]+front[0])/2,(back[1]+front[1])/2]

        enemy_front = cs[BLUE]
        enemy_back = cs[YELLOW]
        enemy = [(enemy_back[0]+enemy_front[0])/2,(enemy_back[1]+enemy_front[1])/2]

        goal = cs[GREEN]
        #goal = [(enemy[0]+ball[0])/2,(enemy[1]+ball[1])/2]
    else:
        front = cs[BLUE]
        back = cs[YELLOW]
        start = [(back[0]+front[0])/2,(back[1]+front[1])/2]

        enemy_front = cs[RED]
        enemy_back = cs[WHITE]
        enemy = [(enemy_back[0]+enemy_front[0])/2,(enemy_back[1]+enemy_front[1])/2]

        goal = [(enemy[0]+ball[0])/2,(enemy[1]+ball[1])/2]
        #goal = cs[GREEN]

    if np.isnan(enemy[0]):
        enemy[0] = -enemyRadius
    if np.isnan(enemy[1]):
        enemy[1] = -enemyRadius

    if np.isnan(ball[0]) or np.isnan(ball[1]):
        ball = lastBall[bot]

    if np.isnan(goal[0]) or np.isnan(goal[1]):
        # Can't see ball
        #print("Can't see ball, going to centre")
        goal = lastBall[bot]
    else:
        lastBall[bot] = goal


    if not(np.isnan(start[0]) or np.isnan(start[1])):
        goalDisp = [0,0]
        startNode = prm.Node(start[0],start[1],N)
        goalNode = prm.Node(goal[0]+goalDisp[0],goal[1]+goalDisp[1],N+1)
        frontNode = prm.Node(front[0],front[1],N)
        enemy = prm.Obstacle(enemy[0],enemy[1],enemyRadius)
        ballNode = prm.Obstacle(ball[0],ball[1],ballRadius)

        #Find path
        x,y = prm.pathPlan(graph,startNode,goalNode,enemy,ballNode,k,avoidBall=False,draw=True,w=640,h=480,frame=frame,bot=bot)
        next = prm.Node(x,y,0)

        '''
        if motion.dist(startNode,goalNode)<enemyRadius:
            #print("RESET NEXT TO GOAL")
            next = prm.Node(goalNode.x,goalNode.y,0)
        '''

        last[bot] = angle[bot]
        lastl[bot] = distance[bot]

        if(turnVel[bot]!=0):
            #Turn to goal
            Il[bot] = 0
            turnVel[bot],angle[bot] = motion.PID_Angle(startNode,frontNode,next,k_angle[bot],I[bot],last[bot],tol,bot)
            #turnVel[bot],angle[bot] = motion.Stochastic_Angle(startNode, frontNode, next, tol, bot)
            if np.isnan(turnVel[bot]):
                print("ANGLE IS NAN")
                turnVel[bot] = 0
            limitSet = [0, 0, 0, 0, 0, turnVel[bot]]
            client.send_to_server(limitSet, servers[bot], sockets[bot]) #Sending sets to servers
            results = client.get_replies(sockets[bot])
        elif velocity[bot]!=0:
            #Move to goal
            I[bot] = 0
            velocity[bot],distance[bot] = motion.PID_Linear(startNode,next,k_linear[bot],Il[bot],lastl[bot],toll,bot)
            #velocity[bot],distance[bot] = motion.Stochastic_Linear(startNode, next, toll, bot)

            if np.isnan(velocity[bot]):
                print("VELOCITY IS NAN")
                velocity[bot] = 0
            limitSet = [velocity[bot], 0, 0, 0, 0, 0]
            client.send_to_server(limitSet, servers[bot], sockets[bot]) #Sending sets to servers
            results = client.get_replies(sockets[bot])
        else:
            #Do nothing
            turnVel[bot] = 3
            velocity[bot] = 3
            #print("turnVel = %i, Velocity = %i" %(turnVel[bot],velocity[bot]))
            #limitSet = [0, 0, 0, 0, 0, 0]

        I[bot] += angle[bot]
        Il[bot] += distance[bot]





if __name__ == "__main__":
    graph = prm.initGraph(N,H,W,Pad,k,manual)
    cap = cv2.VideoCapture(0)

    # turtlebot server 2
    client.create_connections(servers[1], ports[1], sockets[1])
    # turtlebot server 1
    client.create_connections(servers[0], ports[0], sockets[0])


    while(True):

        ret, frame = cap.read() # Capture frame-by-frame
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) # Convert from BGR to RGB
        image = np.array(frame_rgb).astype(np.float)/255 # Convert from Int8 to float
        cs = colour.findCenters(image,tol=0.1,draw=True,frame=frame) # Find colour centres

        run(0)
        run(1)

        cv2.imshow('frame',frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        print("\n")

    client.close_connections(sockets[0]) #Closing all connections to servers.
    client.close_connections(sockets[1])

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()
