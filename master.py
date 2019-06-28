import numpy as np
import matplotlib.pyplot as plt
import cv2
import time
from playsound import playsound

import colour
from colour import RED, WHITE, BLUE, YELLOW, GREEN
import prm
import motion
import client

# server names for turtlebots which we connect to
servers = ["10.199.61.21", "10.199.26.14"]
# ports in which the messages pass through
ports = [12007, 12006]
# sockets for each port/server connection
sockets = [0, 1]
# 0 = Red-White
# 1 = Orange-Yellow

N = 9 # Number of samples for PRM
k = 10  # Number of nearest neighbours for PRM
W = 640 # Width of arena
H = 480 # Height of arena
Pad = 50 # Padding around edges
manual = True

lastBall = [[H/2,W/2],[H/2,W/2]]

# PID Stuff

# anglur kp, ki, kd for each bot
k_angle = [[0.15, 0.025, 0.1], [0.15, 0.025, 0.1]]
# linear velocity kp, ki, kd for each bot
k_linear = [[0.002, 0.0001, 0.005], [0.002, 0.0001, 0.005]]
# stores the erros (angle) for each bot
I = [0, 0]
# stores the errors (distance) for each bot
Il = [0, 0]
# keeps track of the last angle for each bot
last = [0, 0]
# keeps track of the previous distnace for each bot
lastl = [0, 0]

# variables which keep track of angle and distance for each bot
angle = [0, 0]
distance = [0, 0]

tol = 0.25
toll = 0.01
# default angular velocity for each bot
turnVel = [3,3]
# default velocity for each bot
velocity = [3,3]

# padding radius for each object
enemyRadius = 80
ballRadius = 20

def run(bot):
    ball = cs[GREEN]
    # the ball is the goal
    if bot == 0:
        # Define points
        back = cs[WHITE]
        front = cs[RED]
        start = [(back[0]+front[0])/2,(back[1]+front[1])/2]

        enemy_front = cs[BLUE]
        enemy_back = cs[YELLOW]
        enemy = [(enemy_back[0]+enemy_front[0])/2,(enemy_back[1]+enemy_front[1])/2]

        goal = cs[GREEN]
        #goal = [(enemy[0]+ball[0])/2,(enemy[1]+ball[1])/2]

    else: # the midpoint of the line between the ball and the other bot is the goal
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



def queue_music(bot, game_over=False):
    print("Playing music")



if __name__ == "__main__":
    graph = prm.initGraph(N,H,W,Pad,k,manual)
    cap = cv2.VideoCapture(0)
    # number_of_games = 3
    # score = [0, 0]
    # bot_0 = 0
    # bot_1 = 1

    # turtlebot server 2
    client.create_connections(servers[1], ports[1], sockets[1])
    # turtlebot server 1
    client.create_connections(servers[0], ports[0], sockets[0])


    ''' # test game setup
    queue_music(bot, game_over=False)
    for i in range(number_of_games):
        # here we want to set the positions of the bots, giving them 10 seconds to get to their respective positions
        time_reset = time.time() + 10
        while(time.time() < time_reset):
            reset_bots()
            print("Reset Bots")
        time_game = time.time() + 30
        while(time.time() < time_game):
            ret, frame = cap.read() # Capture frame-by-frame
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) # Convert from BGR to RGB
            image = np.array(frame_rgb).astype(np.float)/255 # Convert from Int8 to float
            cs = colour.findCenters(image,tol=0.1,draw=True,frame=frame) # Find colour centres

            # bot 1
            # return either 1 or zero if bot one or not
            score[bot_0] += run(bot_0)
            # bot 2
            # return either 1 or zero if bot one or not
            score[bot_1] += run(bot_1)

            cv2.imshow('frame',frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            print("\n")

    # at the end of the game we show the winner and play their music

    # queue_music(bot, game_over=True)

    # after the number of games played close all connections and windows


    # closing connections to servers
    client.close_connections(sockets[0])
    client.close_connections(sockets[1])

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()


    '''
    while(True):

        ret, frame = cap.read() # Capture frame-by-frame
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) # Convert from BGR to RGB
        image = np.array(frame_rgb).astype(np.float)/255 # Convert from Int8 to float
        cs = colour.findCenters(image,tol=0.1,draw=True,frame=frame) # Find colour centres

        # bot 1
        run(0)
        # bot 2
        run(1)

        cv2.imshow('frame',frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        print("\n")

    # closing connections to servers
    client.close_connections(sockets[0])
    client.close_connections(sockets[1])

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()
