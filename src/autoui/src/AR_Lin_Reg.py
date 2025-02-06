#! /usr/bin/env python

import cv2
import rospy
import pandas as pd
import numpy as np
import time
import math
import threading
import keyboard
from sklearn.linear_model import LinearRegression
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from visualization_msgs.msg import Marker 
from visualization_msgs.msg import MarkerArray 
from geometry_msgs.msg import Point 
import roslib
import tf.transformations
from geometry_msgs.msg import Twist, PolygonStamped

TB_xy = [] # list to store the Turtlebot xy for corresponding screen click point
ScreenXY = [] # list of Screen Click (x pixel , y pixel) vector 
xt = 0.0
yt = 0.0

# Use the previous saved regression and calibration to run Turtlebot PTP

# saved_reg_points = pd.read_csv('/home/i3dlab/catkintry_ws/src/tb3_control/src/RegPointSaved.csv', index_col = 0)
# saved_reg_points[['ScreenX','ScreenY']]
# reg = LinearRegression().fit(saved_reg_points[['ScreenX','ScreenY']], saved_reg_points[['TurtlebotX','TurtlebotY']])
# reg.score(saved_reg_points[['ScreenX','ScreenY']],saved_reg_points[['TurtlebotX','TurtlebotY']])

def newOdom (msg):
    global xt
    global yt
    #p = PolygonStamped()
    x0 = msg.polygon.points[0].x
    y0 = msg.polygon.points[0].y
    x1 = msg.polygon.points[1].x
    y1 = msg.polygon.points[1].y
    x2 = msg.polygon.points[2].x
    y2 = msg.polygon.points[2].y
    x3 = msg.polygon.points[3].x
    y3 = msg.polygon.points[3].y

    v = [ [ x0, y0],
    [ x1, y1],
    [ x2, y2 ],
    [x3, y3] ]

    ans = [0, 0]

    n = len(v)
    signedArea = 0

    # For all vertices
    for i in range(len(v)):

        x0 = v[i][0]
        y0 = v[i][1]
        x1 = v[(i + 1) % n][0]
        y1 =v[(i + 1) % n][1]

        # Calculate value of A
        # using shoelace formula
        A = (x0 * y1) - (x1 * y0)
        signedArea += A

        # Calculating coordinates of
        # centroid of polygon
        ans[0] += (x0 + x1) * A
        ans[1] += (y0 + y1) * A

    signedArea *= 0.5
    xt = (ans[0]) / (6 * signedArea)
    yt = (ans[1]) / (6 * signedArea)
    #print(x,y)
    return

def click_event(event, x, y, flags, params):
    if event == cv2.EVENT_RBUTTONDOWN:
        print("Right clicked")
        print("Screen at pos",x,y)
        print("TB at pos",xt,yt)
        ScreenXY.append((x,y))
        TB_xy.append((xt,yt))
    if event == cv2.EVENT_LBUTTONDOWN:
        print("Left Clicked")
        print("Screen at pos",x,y)
        print("TB at pos",xt,yt)
        #Predict and move turtlebot
    

sub = rospy.Subscriber('/move_base/global_costmap/footprint', PolygonStamped, newOdom)
rospy.init_node('movebase_client')

cap = cv2.VideoCapture(0)
# The device number might be 0 or 1 depending on the device and the webcam
# cap.set(3, 1280)
# cap.set(4, 720)

# variables 
pTime = 0
cTime = 0
global skipframes
# skipframes = True

pausefollow = False

while(len(TB_xy)<10):
    success, img = cap.read()
    frame = img.copy()
    img = cv2.resize(img,(1280,720))
    # print(f"size of img = {img.shape[1]},{img.shape[0]}")

    # Calculating frames per second
    cTime = time.time()
    fps = 1 / (cTime - pTime)
    pTime = cTime

    cv2.putText(img, str(int(fps)), (10, 70), cv2.FONT_HERSHEY_PLAIN, 3,(255, 0, 255), 3)

    # click event for mapping screen xy to dobot xy using linear regression
    # left click for ptp motion and right click to record screen and dobot xy for mapping 
    cv2.namedWindow('Image',cv2.WINDOW_FULLSCREEN)
    cv2.setMouseCallback('Image', click_event)

    cv2.imshow('Image', img)
    if cv2.waitKey(1) == 27:
        break
cap.release()
cv2.destroyAllWindows()

print(TB_xy)
print(ScreenXY)

# save regressionn mapping in csv file
TB_xy_copy = TB_xy.copy()
ScreenXY_copy = ScreenXY.copy()

reg_dobot_points = pd.DataFrame(TB_xy_copy,columns=['TurtlebotX','TurtlebotY'])
reg_screen_points = pd.DataFrame(ScreenXY_copy,columns=['ScreenX','ScreenY'])
reg_points_df = reg_screen_points.join(reg_dobot_points)
print(reg_points_df)
# print(reg_dobot_points)
# print(reg_screen_points)

reg_points_df.to_csv('/home/i3dlab/catkintry_ws/src/tb3_control/src/RegPointSaved.csv')