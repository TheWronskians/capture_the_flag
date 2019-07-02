import numpy as np
import matplotlib.pyplot as plt
import cv2
import time
import math

RED = 0
WHITE = 1
BLUE = 2
YELLOW = 3
GREEN = 4

red = [0.93365103, 0.35130355, 0.28260624] #Light
#red = [0.83925676, 0.19983858, 0.15198655] #Dark
white = [1, 1, 1]
#orange = [0.99148726, 0.5729994, 0.25187913]
#yellow = [0.9834839 , 1, 0.76263994]
#yellow = [0.99455225, 0.9999614 , 0.5426788 ] #brighter
yellow = [0.9834429 , 0.9999806 , 0.62282646]
green = [0.61612934, 0.90352577, 0.5615079 ]
#blue = [0.501899  , 0.89933634, 0.99078834]
blue = [0.38518488, 0.8185951 , 0.96491045]

def displayImage(images, nrows = 1, ncols=1, title=[],image_max=0,sizex=15,sizey=8):
    #Handle the case of 1 image
    if nrows == 1 and ncols == 1:
        images = [images]
    #Mismatch
    if len(images) != nrows*ncols:
        print("Number of images != number of subplots")
        return
    #Title mismathc
    if len(images) != len(title) and len(title)!=0:
        print("Number of images != number of titles")
        return
    fig = plt.figure(figsize=(sizex,sizey))
    ax = []
    for i in range(1, ncols*nrows +1):
        image = images[i-1]

        #Deal for various types
        type = image.dtype
        if np.issubdtype(type, np.integer):
            if image_max==0:
                im_max = np.iinfo(type).max
            else:
                im_max=copy.deepcopy(image_max)
        else:
            im_max = 1

        plt.gray()
        ax.append( fig.add_subplot(nrows, ncols,i))
        if len(title)!=0:
            ax[-1].set_title(title[i-1])
        plt.axis("off")
        plt.imshow(image,vmin=0,vmax=im_max)
    plt.show()
    return ax

def getColours():
    red = plt.imread("red_light.png")
    purple = plt.imread("purple.png")
    blue = plt.imread("blue.png")
    yellow = plt.imread("yellow.png")
    green = plt.imread("green.png")
    #blue = plt.imread("blue.png")
    return [red,purple,blue,yellow,green]

def getAverages(colours):
    avs = []
    for c in colours:
        av = np.mean(c,axis=(0,1))
        avs.append(av[0:3])
    return avs

def distIm(c1,c2):
    return c1-c2

def drawOnFeed(frame,cs):
    avs = [red,white,blue,yellow,green]
    for i in range(len(avs)):
        if not(np.isnan(cs[i][0]) or np.isnan(cs[i][1])):
            #newCol = (int(avs[i][2]*255),int(avs[i][1]*255),int(avs[i][0]*255)) #Reversed because BGR
            newCol = (0,0,0)
            newC = (int(round(cs[i][1])),int(round(cs[i][0]))) #Reversed because image
            cv2.circle(frame,newC,5,newCol,2)

def findCenters(image,tol=0.05,draw=False,frame=None):
    avs = [red,white,blue,yellow,green]
    cs = []
    uR = red+np.ones(3)*tol
    lR = red-np.ones(3)*tol
    uW = white+np.ones(3)*tol
    lW = white-np.ones(3)*tol
    uB = blue+np.ones(3)*tol
    lB = blue-np.ones(3)*tol
    uY = yellow+np.ones(3)*tol
    lY = yellow-np.ones(3)*tol
    uG = green+np.ones(3)*tol
    lG = green-np.ones(3)*tol
    u = [uR,uW,uB,uY,uG]
    l = [lR,lW,lB,lY,lG]
    for i in range(5):
        mask = cv2.inRange(image, l[i], u[i])
        res = cv2.bitwise_and(image,image, mask= mask)
        pix = np.where(mask==255)
        pixels = np.array([pix[0],pix[1]])
        centre = np.mean(pixels,axis=1)
        cs.append(centre)
    if draw and frame is not None:
        drawOnFeed(frame,cs)
    return cs

def calibrate():
    test = plt.imread("test.png")[:,:,0:3]
    colours = getColours()
    testCols = []
    avs = getAverages(colours)
    for av in avs:
        testIm = np.ones((200,200,3))
        for x in range(200):
            for y in range(200):
                testIm[x,y,:] = av
        testCols.append(testIm)
    cs = findCenters(test)
    dispCols = colours+testCols
    displayImage(dispCols,2,5)
    fig, ax = plt.subplots()
    ax.imshow(test)
    for c in cs:
        ax.scatter(c[1], c[0], s=50, color='cyan')
    plt.show()
    print(avs)

if __name__ == "__main__":
    calibrate()
