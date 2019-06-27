import numpy as np
import matplotlib.pyplot as plt
import cv2
import time

RED = 0
WHITE = 1
ORANGE = 2
YELLOW = 3
GREEN = 4
BLUE = 5

red = [0.93365103, 0.35130355, 0.28260624]
white = [1, 1, 1]
orange = [0.99148726, 0.5729994, 0.25187913]
yellow = [0.9834839 , 1, 0.76263994]
green = [0.61612934, 0.90352577, 0.5615079 ]
blue = [0.28260624, 0.35130355, 0.93365103]

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
    red = plt.imread("red.png")
    purple = plt.imread("purple.png")
    orange = plt.imread("orange.png")
    yellow = plt.imread("yellow.png")
    green = plt.imread("green.png")
    blue = plt.imread("blue.png")
    return [red,purple,orange,yellow,green,blue]

def getAverages(colours):
    avs = []
    for c in colours:
        av = np.mean(c,axis=(0,1))
        avs.append(av[0:3])
    return avs

def distIm(c1,c2):
    return c1-c2

def findCenters(image,tol=0.05):
    avs = [red,white,orange,yellow,green,blue]
    cs = []
    uR = red+np.ones(3)*tol
    lR = red-np.ones(3)*tol
    uW = white+np.ones(3)*tol
    lW = white-np.ones(3)*tol
    uO = orange+np.ones(3)*tol
    lO = orange-np.ones(3)*tol
    uY = yellow+np.ones(3)*tol
    lY = yellow-np.ones(3)*tol
    uG = green+np.ones(3)*tol
    lG = green-np.ones(3)*tol
    uB = blue+np.ones(3)*tol
    lB = blue-np.ones(3)*tol
    u = [uR,uW,uO,uY,uG,uB]
    l = [lR,lW,lO,lY,lG,lB]
    for i in range(5):
        mask = cv2.inRange(image, l[i], u[i])
        res = cv2.bitwise_and(image,image, mask= mask)
        pix = np.where(mask==255)
        pixels = np.array([pix[0],pix[1]])
        centre = np.mean(pixels,axis=1)
        cs.append(centre)
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

if __name__ == "__main__":
    calibrate()
