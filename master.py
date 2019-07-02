import numpy as np
import matplotlib.pyplot as plt
import cv2
import time
import winsound

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

score = [0, 0]

startPoints = [[380,100],[380,540]]

# PID Stuff

# angular kp, ki, kd for each bot
#k_angle = [[0.15, 0.025, 0.1], [0.15, 0.025, 0.1]]
k_angle = [[0.15, 0.025, 0.1], [0.15, 0.025, 0.15]]
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

font = cv2.FONT_HERSHEY_SIMPLEX
topLeft = (20,50)
topRight = (420,50)
fontScale = 1
fontColor = (255,255,255)
lineType = 2

def run(bot,reset=False,game_over=False):
    ball = cs[GREEN]
    canSeeBall = True
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

    if reset:
        goal = startPoints[bot]

    if game_over:
        goal = [H/2,W/2]

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
        canSeeBall = False
    else:
        lastBall[bot] = goal


    if not(np.isnan(start[0]) or np.isnan(start[1])):

        goalDisp = [0,0]
        startNode = prm.Node(start[0],start[1],N)
        goalNode = prm.Node(goal[0]+goalDisp[0],goal[1]+goalDisp[1],N+1)
        frontNode = prm.Node(front[0],front[1],N)
        enemy = prm.Obstacle(enemy[0],enemy[1],enemyRadius)
        ballNode = prm.Obstacle(ball[0],ball[1],ballRadius)

        if not(reset and motion.dist(startNode,goalNode)<70):
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

            if bot==0 and motion.dist(startNode,ballNode)<70 and canSeeBall:
                return True
    return False

def queue_music(song_name):
    winsound.PlaySound('music/'+song_name+'.wav', winsound.SND_ASYNC)

if __name__ == "__main__":
    graph = prm.initGraph(N,H,W,Pad,k,manual)
    cap = cv2.VideoCapture(0)
    number_of_games = 3
    # bot_0 = 0
    # bot_1 = 1

    directory = "footage/"
    format = ".avi"
    timestamp = time.ctime(time.time()).split(" ")[4].split(":")
    filename = "master-"+timestamp[0]+"-"+timestamp[1]+"-"+timestamp[2]

    fourcc = cv2.VideoWriter_fourcc('M','J','P','G')
    out = cv2.VideoWriter(directory+filename+format, fourcc, 20.0, (int(cap.get(3)),int(cap.get(4))))

    # turtlebot servers
    client.create_connections(servers[0], ports[0], sockets[0])
    client.create_connections(servers[1], ports[1], sockets[1])

    queue_music('eye_of_the_tiger')
    for i in range(number_of_games):
        print("Game %i" %(i))
        reset_time = time.time()+20
        while(time.time()<reset_time):
            ret, frame = cap.read() # Capture frame-by-frame
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) # Convert from BGR to RGB
            image = np.array(frame_rgb).astype(np.float)/255 # Convert from Int8 to float
            cs = colour.findCenters(image,tol=0.1,draw=True,frame=frame) # Find colour centres

            run(0,reset=True)
            run(1,reset=True)

            time_left = round(reset_time-time.time())

            cv2.putText(frame,str(time_left), topLeft, font, fontScale,fontColor,lineType)
            cv2.putText(frame,str(score), topRight, font, fontScale,fontColor,lineType)

            cv2.imshow('frame',frame)
            out.write(frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        game_time = time.time()+30
        by_ball = False
        while(time.time()<game_time):

            ret, frame = cap.read() # Capture frame-by-frame
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) # Convert from BGR to RGB
            image = np.array(frame_rgb).astype(np.float)/255 # Convert from Int8 to float
            cs = colour.findCenters(image,tol=0.1,draw=True,frame=frame) # Find colour centres

            by_ball = run(0)
            run(1)

            time_left = round(game_time-time.time())

            cv2.putText(frame,str(time_left), topLeft, font, fontScale,fontColor,lineType)
            cv2.putText(frame,str(score), topRight, font, fontScale,fontColor,lineType)

            cv2.imshow('frame',frame)
            out.write(frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            if by_ball:
                score[0] += 1
                break
        if not by_ball:
            score[1] += 1
    winsound.PlaySound(None, winsound.SND_ASYNC)
    victory_time = time.time()+15
    victory = [False,False]

    if score[0]>score[1]:
        victory[0]=True
        victor = 0
        queue_music('rocky')
    else:
        victory[1]=True
        victor = 1
        queue_music('john_cena')
    while(time.time()<victory_time):
        ret, frame = cap.read() # Capture frame-by-frame
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) # Convert from BGR to RGB
        image = np.array(frame_rgb).astype(np.float)/255 # Convert from Int8 to float
        cs = colour.findCenters(image,tol=0.1,draw=True,frame=frame) # Find colour centres

        run(0,reset=True,game_over=victory[0])
        run(1,reset=True,game_over=victory[1])

        #time_left = round(reset_time-time.time())

        #cv2.putText(frame,str(time_left), topLeft, font, fontScale,fontColor,lineType)
        cv2.putText(frame,str(score), topRight, font, fontScale,fontColor,lineType)

        cv2.imshow('frame',frame)
        out.write(frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    ttime = time.time()+20
    while(time.time()<ttime):
        ret, frame = cap.read() # Capture frame-by-frame
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) # Convert from BGR to RGB
        image = np.array(frame_rgb).astype(np.float)/255 # Convert from Int8 to float
        cs = colour.findCenters(image,tol=0.1,draw=True,frame=frame) # Find colour centres


        limitSet = [0, 0, 0, 0, 0, 1]
        client.send_to_server(limitSet, servers[victor], sockets[victor]) #Sending sets to servers
        results = client.get_replies(sockets[victor])

        cv2.putText(frame,str(score), topRight, font, fontScale,fontColor,lineType)

        cv2.imshow('frame',frame)
        out.write(frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    winsound.PlaySound(None, winsound.SND_ASYNC)
    # closing connections to servers
    client.close_connections(sockets[0])
    client.close_connections(sockets[1])

    # When everything done, release the capture
    cap.release()
    out.release()
    cv2.destroyAllWindows()
