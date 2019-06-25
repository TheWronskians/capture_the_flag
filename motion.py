import numpy as np
import matplotlib.pyplot as plt
from prm import *
import math

def getAngle(start,front,goal):
    b = start
    a = goal
    c = front
    ang = math.atan2(c.y-b.y, c.x-b.x) - math.atan2(a.y-b.y, a.x-b.x)
    return -ang

def drawScene(start,front,goal):
    sfx = [start.x,front.x]
    sfy = [start.y,front.y]
    sgx = [start.x,goal.x]
    sgy = [start.y,goal.y]
    plt.plot(sfx,sfy,c='green')
    plt.plot(sgx,sgy,c='red')

def turnToGoal(start,front,goal,kp,ki,I):
    angle = getAngle(start,front,goal)
    turnVel = kp*angle #P control
    print(math.degrees(angle),turnVel)
    I += ki*np.abs(angle) #I Control
    #turnVel += I
    return turnVel,I


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
        newFx = start.x-np.cos(turnVel)
        newFy = start.y-np.sin(turnVel)
        front = Node(newFx,newFy,0)
    print("Angle = ",getAngle(start,front,next))
    plt.show()
