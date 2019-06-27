import numpy as np
import matplotlib.pyplot as plt
from prm import *
import math

def getAngle(start,front,goal):
    b = start
    a = goal
    c = front
    ang = math.atan2(c.y-b.y, c.x-b.x) - math.atan2(a.y-b.y, a.x-b.x)
    if ang<-math.pi:
        ang+=2*math.pi
    elif ang>math.pi:
        ang-=2*math.pi
    return ang

def dist(a,b):
    return np.sqrt((a.x-b.x)**2 + (a.y-b.y)**2)

def drawScene(start,front,goal):
    sfx = [start.x,front.x]
    sfy = [start.y,front.y]
    sgx = [start.x,goal.x]
    sgy = [start.y,goal.y]
    plt.plot(sfx,sfy,c='green')
    plt.plot(sgx,sgy,c='red')

def turnToGoal(angle,start,front,goal,kp,ki=0):
    turnVel = -kp*angle #P control
    print(math.degrees(angle),turnVel)
    I += ki*np.abs(angle) #I Control
    #turnVel += I
    return turnVel

def PID_Angle(start,front,goal,k,I,last,tol):
    angle = getAngle(start,front,goal)
    I += angle
    D = angle-last
    #print(angle)
    if np.abs(angle-math.pi)<tol:
        return 0, angle
    else:
        turn = k[0]*angle + k[1]*I + k[2]*D
        return turn,angle

# stochastic approach to pid for the angle
def Stochastic_Angle(start, front, goal, prev_angle, tol):
    angle = getAngle(start, front, goal)
    if np.abs(angle-math.pi) < tol:
        return 0
    else:
        # turn = (float(goal - angle)) / 4.0
        turn  = angle / 4.0
        return turn


def PID_Linear(start,goal,k,I,last,tol):
    d = dist(start,goal)
    I += d
    D = d-last
    #print(angle)
    if np.abs(d)<tol:
        return 0,d
    else:
        velocity = k[0]*d + k[1]*I + k[2]*D
        return velocity,d

# stochastic approach to pid for velocity
def Stochastic_Linear(start, goal, tol):
    distance = (float(goal - start))
    if np.abs(distance) < tol:
        return 0
    else:
        velocity = distance / 4.0
        return velocity


if __name__ == "__main__":
    N = 10
    k = 30
    wallPad = 10
    w = 100
    h = 100
    graph = initGraph(N,w,h,wallPad,k)
    start = Node(np.random.randint(0,w-wallPad),np.random.randint(0,h-wallPad),N)
    front = Node(start.x,start.y+2,N+2)
    goal = Node(np.random.randint(0,w-wallPad),np.random.randint(0,h-wallPad),N+1)
    enemy = Obstacle(np.random.randint(0,w-wallPad),np.random.randint(0,h-wallPad),10)
    ball = Obstacle(np.random.randint(0,w-wallPad),np.random.randint(0,h-wallPad),5)
    x,y = pathPlan(graph,start,goal,enemy,ball,k,True,draw=False)
    next = Node(x,y,0)
    plt.axis()
    kp = 0.5
    while True:
        drawScene(start,front,goal)
        plt.pause(0.05)
        I = 0
        turnVel,I = turnToGoal(start,front,next,kp,0.001,I)
        if np.abs(turnVel)<kp*0.05:
            break
        newFx = start.x-2*np.cos(turnVel)
        newFy = start.y-2*np.sin(turnVel)
        front = Node(newFx,newFy,0)
    print("Angle = ",getAngle(start,front,next))
    plt.show()
