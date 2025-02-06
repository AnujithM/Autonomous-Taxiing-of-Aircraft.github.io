# -*- coding: utf-8 -*-
"""
Created on Wed Feb  1 16:16:40 2023

@author: prash
"""


import numpy as np
import cv2, os
import matplotlib.image as mpimg
# from scipy.misc import imresize
# from moviepy.editor import VideoFileClip
#from IPython.display import HTML
# import tensorflow.keras as keras
# from keras.models import model_from_json
# from tensorflow.keras.models import model_from_json
# from tensorflow.keras.models import load_model
from scipy.optimize import curve_fit
import matplotlib.pyplot as plt
from glob import glob
from time import time
import csv
import socket
import pandas as pd

grayMavg = [1,2,3]
# hsvlog = pd.read_csv("hsvlog.csv")
def getControl(img, hsvlog, rect_width=50, rect_height=100, lw=20, st_threshold=0):
    img_shape = img.shape[:2]
    rect_xpos = int((img_shape[0] - rect_width) / 2)
    rect_ypos = img_shape[1] - rect_height
    centredot_ypos = int(rect_ypos + rect_height / 2)
    mask1 = np.zeros(img_shape, dtype="uint8")
    cv2.rectangle(mask1, (rect_xpos, rect_ypos), (rect_xpos + rect_width, rect_ypos + rect_height), 255, -1)
    
    kernel = np.ones((5, 5), np.uint8)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lowArray = list(hsvlog.iloc[len(hsvlog)-1, 1:4])
    highArray = list(hsvlog.iloc[len(hsvlog)-1, 4:])
    HSVLOW = np.array(lowArray).astype(np.uint8)
    HSVHIGH = np.array(highArray).astype(np.uint8)
    mask = cv2.inRange(hsv, HSVLOW, HSVHIGH)
    cv2.imwrite("mask.png", mask)
    mask = cv2.erode(mask, kernel, iterations=1)
    mask = cv2.dilate(mask, kernel, iterations=2)
    cv2.imwrite("mask1.png", mask)
    mask = cv2.bitwise_and(mask, mask, mask=mask1)
    cv2.imwrite("mask2.png", mask)
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)
    if len(contours) > 0:
        allContours = contours[0][0]

        for i in range(len(contours)):
            allContours = np.vstack((allContours, np.reshape(contours[i], (contours[i].shape[0], 2))))
        x = allContours[:,0]
        y = allContours[:,1]
    else:
        return mask, "None"
    try:
        popt, pcov = curve_fit(func, y, x)
    except:
        return mask, "None"
    y_fit = rect_height - 20
    x_line = func(y_fit, *popt)
    x_line = x_line.astype(int)
    p1 = (x_line, int(y_fit + lw/2))
    p2 = (x_line, int(y_fit - lw/2))
    rect_centre = (int(rect_xpos + rect_width / 2), int(rect_ypos + rect_height / 2))
    
    cv2.line(mask, p1, p2, (255,255,255))
    cv2.circle(mask, rect_centre, radius=1, color=(0,0,255), thickness=1)
    cv2.rectangle(mask, (rect_xpos, rect_ypos), (rect_xpos + rect_width, rect_ypos + rect_height), 255, 1)
    if((rect_centre[0] - st_threshold) <= x_line <= (rect_centre[0] + st_threshold)):
        control = 0
    elif x_line < rect_centre[0]:
        control = rect_centre[0] - x_line
    else:
        control = rect_centre[0] - x_line
        
    
    return mask, control


def func(x, a, b, c):
    return a*x**2 + b*x + c

def startLightControl(connect, hsvlog):
    
    # define a video capture object
    vid = cv2.VideoCapture(0)
    vid.set(3, 1920)
    vid.set(4, 1080)
    font = cv2.FONT_HERSHEY_SIMPLEX
    while(vid.isOpened()):
          
        # Capture the video frame
        # by frame
        ret, frame = vid.read()
        frame = cv2.GaussianBlur(frame, (5, 5), 0)
        img = cv2.resize(frame, (224,224))
        mask , control = getControl(img, hsvlog, rect_width=img.shape[0], rect_height=img.shape[1])
        if connect:
            sock.sendto(str(control).encode('utf-8'), serverAddressPort)
        width = frame.shape[1]
        height = frame.shape[0]
        mask = cv2.resize(mask, frame.shape[:2][::-1])
        blanks = np.zeros_like(mask).astype(np.uint8)
        
        cv2.putText(mask, 'Control:' + str(control), (int((width / 2) + 250), int((height / 2) + 200)), font,
                    2,
                    (255, 255, 255), 4,
                    cv2.LINE_AA)
        result = cv2.addWeighted(frame, 0.7, mask, 1, 0)
        # Display the resulting frame
        cv2.imshow('frame', result)
        
        # the 'q' button is set as the
        # quitting button you may use any
        # desired button of your choice
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
      
    # After the loop release the cap object
    vid.release()
    # Destroy all the windows
    cv2.destroyAllWindows()
if __name__ == "__main__":
    hsvlog = pd.read_csv("hsvlog2.csv")
    connect = False
    if connect:
        UDP_IP = "192.168.0.35"#"192.168.43.69"
        UDP_PORT = 20001
        print(3)
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        print(2)
        serverAddressPort = (UDP_IP, UDP_PORT)
        print(1)
        sock.sendto("Hi Server".encode('utf-8'), serverAddressPort)
        print('after sendto')
        data, addr = sock.recvfrom(1024)
        print(data.decode('utf-8'), "was recieved")
    startLightControl(connect, hsvlog)
    