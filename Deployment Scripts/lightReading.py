# -*- coding: utf-8 -*-
"""
Created on Wed May 10 17:37:32 2023

@author: prash
"""

import cv2
import numpy as np

# define a video capture object
vid = cv2.VideoCapture(1)
# vid.set(3,1920)
# vid.set(4,1080)
  # font
font = cv2.FONT_HERSHEY_SIMPLEX
  
# org
org = (50, 50)
  
# fontScale
fontScale = 1
   
# Blue color in BGR
color = (255, 0, 0)
  
# Line thickness of 2 px
thickness = 2
mAvgLen = 30
grayMavg = np.zeros(mAvgLen)
blueMavg = np.zeros(mAvgLen)
redMavg = np.zeros(mAvgLen)
greenMavg = np.zeros(mAvgLen)
i = 0
while(True):
      
    # Capture the video frame
    # by frame
    ret, frame = vid.read()
    grayscale = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    grayMean = np.mean(grayscale)
    blueMean = np.mean(frame[:,:,0])
    greenMean = np.mean(frame[:,:,1])
    redMean = np.mean(frame[:,:,2])
    grayMavg[i%mAvgLen] = grayMean
    blueMavg[i%mAvgLen] = blueMean
    greenMavg[i%mAvgLen] = greenMean
    redMavg[i%mAvgLen] = redMean
    # Using cv2.putText() method
    cv2.putText(frame, 'Gray : ' + str(np.mean(grayMavg)), org, font, 
                   fontScale, color, thickness, cv2.LINE_AA)
    cv2.putText(frame, 'blue : ' + str(np.mean(blueMavg)), (50, 100), font, 
                   fontScale, color, thickness, cv2.LINE_AA)
    cv2.putText(frame, 'green : ' + str(np.mean(greenMavg)), (50, 150), font, 
                   fontScale, color, thickness, cv2.LINE_AA)
    cv2.putText(frame, 'red : ' + str(np.mean(redMavg)), (50, 200), font, 
                   fontScale, color, thickness, cv2.LINE_AA)
    # Display the resulting frame
    cv2.imshow('frame', frame)
      
    # the 'q' button is set as the
    # quitting button you may use any
    # desired button of your choice
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    i+=1
  
# After the loop release the cap object
vid.release()
# Destroy all the windows
cv2.destroyAllWindows()