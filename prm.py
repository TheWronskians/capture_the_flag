import numpy as np
import matplotlib.pyplot as plt
import networkx as nx
import copy
import math
import cv2

INF = 1000000

class Node:
    def __init__(self,x,y,i):
        self.x = x
        self.y = y
        self.i = i

    def getX(self):
        return self.x

    def getY(self):
        return self.y

    def getI(self):
        return self.i

class Obstacle:
    def __init__(self,x,y,radius):
        self.x = x
        self.y = y
        self.radius = radius

    def getRadius(self):
        return self.radius

    def getX(self):
        return self.x

    def getY(self):
        return self.y

class Graph:
    def __init__(self,V,adjacency):
        self.V = V
        self.adjacency = adjacency
        for i in range(adjacency.shape[0]):
            self.adjacency[i,i] = 0

    def add(self,node):
        self.V.append(node)

    def reset(self):
        N = self.adjacency.shape[0]
        twoRowINF = np.ones((2,N))*INF
        self.adjacency[N-2:,:] = twoRowINF
        self.adjacency[:,N-2:] = twoRowINF.T
        self.adjacency[N-2,N-2] = 0
        self.adjacency[N-1,N-1] = 0
        del self.V[N-1]
        del self.V[N-2]

    def pruneEdge(self,i,j):
        self.adjacency[i,j] = INF
        self.adjacency[j,i] = INF



def dist(x1,y1,x2,y2):
    return np.sqrt((x1-x2)**2 + (y1-y2)**2)

def printAdjacency(adjacency):
    for r in range(adjacency.shape[0]):
        text = ""
        for c in range(adjacency.shape[1]):
            if adjacency[r,c]==INF:
                text += "&&  "
            else:
                text += str(int(adjacency[r,c]))+"  "
        print(text)

def drawGraph(graph,enemy,ball,adjust=2,path=None,extras=None):
    gr = nx.Graph()
    pos = {}
    for i in range(graph.adjacency.shape[0]-adjust):
        pos[i]=(graph.V[i].x,graph.V[i].y)
        for j in range(graph.adjacency.shape[1]-adjust):
            value = graph.adjacency[i,j]
            if value!=0 and value!=INF:
                gr.add_edge(i,j,weight=int(value))

    nx.draw_networkx(gr,pos)
    labels = nx.get_edge_attributes(gr,'weight')
    nx.draw_networkx_edge_labels(gr,pos,edge_labels=labels)
    if adjust==0:
        pointRadius = 1000
        start = graph.V[graph.adjacency.shape[0]-2]
        goal = graph.V[graph.adjacency.shape[0]-1]
        plt.scatter(start.x,start.y,s=pointRadius,c='red')
        plt.scatter(goal.x,goal.y,s=pointRadius,c='cyan')

    #Draw Path
    if path is not None:
        xs = []
        ys = []
        for i in range(len(path)):
            xs.append(graph.V[path[i]].x)
            ys.append(graph.V[path[i]].y)
        plt.plot(xs,ys,linewidth=5,c='red')

    #Draw obstacles
    for obstacle in [enemy,ball]:
        top = obstacle.x+obstacle.radius
        bot = obstacle.x-obstacle.radius
        left = obstacle.y+obstacle.radius
        right = obstacle.y-obstacle.radius
        tbX = [top,bot]
        tbY = [obstacle.y,obstacle.y]
        lrX = [obstacle.x,obstacle.x]
        lrY = [left,right]
        plt.plot(tbX,tbY,linewidth=2,c='purple')
        plt.plot(lrX,lrY,linewidth=2,c='purple')
        plt.scatter(obstacle.x,obstacle.y,s=500,c='purple')

    if extras is not None:
        cs = ['yellow','orange']
        i = 0
        for ex in extras:
            plt.scatter(ex.x,ex.y,s=500,c=cs[i])
            i = (i+1)%2

    plt.axis("off")
    plt.show()

def nearestNeighbours(graph,i,k,robotRadius=15):
    ds  = []
    for j in range(len(graph.V)):
        d = dist(graph.V[i].x,graph.V[i].y,graph.V[j].x,graph.V[j].y)
        if j==i:
            ds.append(INF)
        else:
            ds.append(d)

    ind = np.argsort(ds)
    return ind[:k]

def initGraph(numSamples,width,height,wallPad,k=10,manual=False):
    '''
    Initialises the graph's adjacency matrix without taking into account obstacles
    or starting/goal positions.
    '''
    empty = []

    graph = Graph(empty,np.ones((numSamples+2,numSamples+2))*INF)
    #Add nodes
    #'''
    if manual:
        i = 0
        for y in [120,320,520]:
            for x in [120,240,360]:
                node = Node(x,y,i)
                graph.add(node)
                i+=1
    else:
        for i in range(numSamples):
            x = np.random.randint(wallPad,width-wallPad)
            y = np.random.randint(wallPad,height-wallPad)
            node = Node(x,y,i)
            graph.add(node)
    #'''
    '''
    N = math.ceil(math.sqrt(numSamples))
    offset = 20
    padW = width-wallPad-offset
    padH = height-wallPad-offset
    X = np.linspace(wallPad+offset,padW,N)
    Y = np.linspace(wallPad+offset,padH,N)
    i = 0
    for x in X:
        for y in Y:
            node = Node(x,y,i)
            graph.add(node)
            i+=1
            '''
    #Get k nearest neighbours
    for node in graph.V:
        neighbours = nearestNeighbours(graph,node.i,k)
        for n in neighbours:
            d = dist(node.x,node.y,graph.V[n].x,graph.V[n].y)
            graph.adjacency[node.i,n] = d
            graph.adjacency[n,node.i] = d

    return graph

def dijkstra(graph,start,goal):
    N = graph.adjacency.shape[0]
    visited = np.zeros(N)
    prev = np.ones(N).astype(np.int)*start.i
    shortest = np.zeros(N)
    for i in range(N):
        shortest[i] = graph.adjacency[i,start.i]
    visited[start.i] = 1
    shortest[start.i] = 0
    for i in range(N-1):
        min = INF
        minv = -1
        for j in range(N):
            if visited[j]==0 and shortest[j]<min:
                min = shortest[j]
                minv = j
        visited[minv] = 1
        for j in range(N):
            if visited[j]==0 and min + graph.adjacency[j,minv]<shortest[j]:
                shortest[j] = min + graph.adjacency[j,minv]
                prev[j] = minv
    return getPath(prev,start,goal)

def getPath(prev,start,goal):
    curr = goal.i
    path = [curr]
    while curr!=start.i:
        curr = prev[curr]
        path.append(curr)
    return path[::-1]

def pruneEdges(graph,obstacle):
    N = graph.adjacency.shape[0]
    for i in range(N):
        for j in range(N):
            if graph.adjacency[i,j]!=0 and graph.adjacency[i,j]!=INF:
                d = graph.adjacency[i,j]
                p = graph.V[i]
                q = graph.V[j]
                M = np.abs(((obstacle.x-p.x)*(q.y-p.y) - (obstacle.y-p.y)*(q.x-p.x))/d)
                if M<=obstacle.radius:
                    graph.pruneEdge(i,j)

def drawOnFeed(graph,frame,path,enemy=None,ball=None,bot=0):
    N = len(graph.V)
    #Draw enemy and ball
    if enemy is not None and not (np.isnan(enemy.y) or np.isnan(enemy.x)):
        enemyPos = (int(round(enemy.y)),int(round(enemy.x)))
        cv2.circle(frame,enemyPos,enemy.radius,(180,105,255),-1)

    if ball is not None and not (np.isnan(ball.y) or np.isnan(ball.x)):
        ballPos = (int(round(ball.y)),int(round(ball.x)))
        cv2.circle(frame,ballPos,ball.radius,(255,255,224),-1)

    #Draw centres
    for v in graph.V:
        if v.i == N-2:
            color = (0,255,0) #Start
        elif v.i == N-1:
            color = (0,0,255) #Goal
        else:
            color = (255,0,0) #Anything else
        newC = (int(round(v.y)),int(round(v.x)))
        cv2.circle(frame,newC,5,color,-1)

    #Draw Path
    for i in range(len(path)-1):
        p1 = (graph.V[path[i]].y,graph.V[path[i]].x)
        p2 = (graph.V[path[i+1]].y,graph.V[path[i+1]].x)
        newP1 = (int(round(p1[0])),int(round(p1[1])))
        newP2 = (int(round(p2[0])),int(round(p2[1])))
        if bot==0:
            pathColor = (255,0,0)
        else:
            pathColor = (255,0,255)
        cv2.line(frame,newP1,newP2,pathColor,1)




def pathPlan(graph,start,goal,enemy,ball,k,avoidBall=True,draw=False,w=640,h=480,frame=None,bot=0):
    G = copy.deepcopy(graph)
    N = G.adjacency.shape[0]
    G.add(start)
    G.add(goal)

    #Get k nearest neighbours
    for node in [start,goal]:
        neighbours = nearestNeighbours(G,node.i,k)
        for n in neighbours:
            d = dist(node.x,node.y,G.V[n].x,G.V[n].y)
            G.adjacency[node.i,n] = d
            G.adjacency[n,node.i] = d

    #Make start and goal neighbours
    if G.adjacency[start.i,goal.i] == INF:
        d = dist(start.x,start.y,goal.x,goal.y)
        G.adjacency[start.i,goal.i] = d
        G.adjacency[goal.i,start.i] = d

    #Prune edges that go through enemy or ball
    pruneEdges(G,enemy)
    if avoidBall:
        pruneEdges(G,ball)

    #Shortest path from start to goal
    path = dijkstra(G,start,goal)
    if draw:
        if frame is not None:
            drawOnFeed(G,frame,path,enemy,ball,bot)
        else:
            drawGraph(G,enemy,ball,0,path)
    myPath = [G.V[path[1]].x,G.V[path[1]].y]
    '''
    if myPath[0]==start.x or myPath[1]==start.y:
        myPath = [h/2,w/2]
    '''
    return myPath[0],myPath[1] #Return next node
    #return G.V[path[-1]].x,G.V[path[-1]].y #Return last node

if __name__ == "__main__":
    N = 100
    k = 30
    wallPad = 10
    w = 100
    h = 100
    graph = initGraph(N,w,h,wallPad,k)
    start = Node(np.random.randint(0,w-wallPad),np.random.randint(0,h-wallPad),N)
    goal = Node(np.random.randint(0,w-wallPad),np.random.randint(0,h-wallPad),N+1)
    enemy = Obstacle(np.random.randint(0,w-wallPad),np.random.randint(0,h-wallPad),10)
    ball = Obstacle(np.random.randint(0,w-wallPad),np.random.randint(0,h-wallPad),5)
    x,y = pathPlan(graph,start,goal,enemy,ball,k,True,draw=True,w=640,h=480)
    '''
    while True:
        x,y = pathPlan(graph,start,goal,enemy,ball,k,True)
        print(x,y)
        start = Node(x,y,N)
        goal = Node(np.random.randint(0,w),np.random.randint(0,h),N+1)
        enemy = Obstacle(np.random.randint(0,w),np.random.randint(0,h),10)
        '''
