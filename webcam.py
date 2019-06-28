import numpy as np
import cv2
import matplotlib.pyplot as plt

import prm
import colour
from colour import GREEN, BLUE

cap = cv2.VideoCapture(0)

N = 100
k = 10
wallPad = 100
h = 640
w = 480
graph = prm.initGraph(N,w,h,wallPad,k)
start = prm.Node(np.random.randint(0,w-wallPad),np.random.randint(0,h-wallPad),N)
goal = prm.Node(np.random.randint(0,w-wallPad),np.random.randint(0,h-wallPad),N+1)
enemy = prm.Obstacle(w/2,h/2,60)
ball = prm.Obstacle(np.random.randint(0,w-wallPad),np.random.randint(0,h-wallPad),10)
while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()

    # Our operations on the frame come here
    #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    array_values = np.array(frame)
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) # Convert from BGR to RGB
    image = np.array(frame_rgb).astype(np.float)/255 # Convert from Int8 to float
    cs = colour.findCenters(image,tol=0.1,draw=True,frame=frame) # Find colour centres
    #print(array_values.shape)
    # Display the resulting frame

    if not (np.isnan(cs[GREEN][0]) or np.isnan(cs[GREEN][1])):
        start = prm.Node(cs[GREEN][0],cs[GREEN][1],N)
    if not (np.isnan(cs[BLUE][0]) or np.isnan(cs[BLUE][1])):
        ball = prm.Obstacle(cs[BLUE][0],cs[BLUE][1],10)


    x,y = prm.pathPlan(graph,start,goal,enemy,ball,k,True,draw=True,w=640,h=480,frame=frame)

    cv2.imshow('frame',frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
