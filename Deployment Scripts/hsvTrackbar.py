import cv2 as cv
import numpy as np
import os
import pandas as pd
import time

logFile = r"hsvLog2.csv"
if os.path.isfile(logFile):
    hsvLog = pd.read_csv(logFile)
else:
    hsvLog = pd.DataFrame(columns=["Time", "LH", "LS", "LV", "HH", "HS", "HV"])

# optional argument for trackbars
def nothing(x):
    pass

# named ites for easy reference
barsWindow = 'Bars'
hl = 'H Low'
hh = 'H High'
sl = 'S Low'
sh = 'S High'
vl = 'V Low'
vh = 'V High'

# set up for video capture on camera 0
cap = cv.VideoCapture(0)
cap.set(3, 1920)
cap.set(4, 1080)

# create window for the slidebars
cv.namedWindow(barsWindow, flags = cv.WINDOW_AUTOSIZE)

# create the sliders
cv.createTrackbar(hl, barsWindow, 0, 179, nothing)
cv.createTrackbar(hh, barsWindow, 0, 179, nothing)
cv.createTrackbar(sl, barsWindow, 0, 255, nothing)
cv.createTrackbar(sh, barsWindow, 0, 255, nothing)
cv.createTrackbar(vl, barsWindow, 0, 255, nothing)
cv.createTrackbar(vh, barsWindow, 0, 255, nothing)

# set initial values for sliders
cv.setTrackbarPos(hl, barsWindow, 0)
cv.setTrackbarPos(hh, barsWindow, 179)
cv.setTrackbarPos(sl, barsWindow, 0)
cv.setTrackbarPos(sh, barsWindow, 255)
cv.setTrackbarPos(vl, barsWindow, 0)
cv.setTrackbarPos(vh, barsWindow, 255)

while(True):
    ret, frame = cap.read()
    frame = cv.resize(frame, (224,224))
    frame = cv.GaussianBlur(frame, (5, 5), 0)

    
    # convert to HSV from BGR
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

    # read trackbar positions for all
    hul = cv.getTrackbarPos(hl, barsWindow)
    huh = cv.getTrackbarPos(hh, barsWindow)
    sal = cv.getTrackbarPos(sl, barsWindow)
    sah = cv.getTrackbarPos(sh, barsWindow)
    val = cv.getTrackbarPos(vl, barsWindow)
    vah = cv.getTrackbarPos(vh, barsWindow)

    # make array for final values
    HSVLOW = np.array([hul, sal, val])
    HSVHIGH = np.array([huh, sah, vah])
    # HSVLOW = np.array([46,	0,	142])
    # HSVHIGH = np.array([82,	255,	255])

    # apply the range on a maskq
    kernel = np.ones((5, 5), np.uint8)
    mask = cv.inRange(hsv, HSVLOW, HSVHIGH)
    mask = cv.erode(mask, kernel, iterations=1)
    mask = cv.dilate(mask, kernel, iterations=2)
    maskedFrame = cv.bitwise_and(frame, frame, mask = mask)

    # display the camera and masked images
    cv.imshow('Masked', maskedFrame)
    cv.imshow('Camera', frame)
	# check for q to quit program with 5ms delay
    if cv.waitKey(1) & 0xFF == ord('q'):
        break
    elif cv.waitKey(1) & 0xFF == ord('s'):
        hsvLog.loc[len(hsvLog)] = [time.strftime("%d/%m/%Y %H:%M:%S"), hul, sal, val, huh, sah, vah]
        print("HSV LOG saved")

# clean up our resources
cap.release()
cv.destroyAllWindows()
hsvLog.to_csv("hsvlog2.csv", index=False)