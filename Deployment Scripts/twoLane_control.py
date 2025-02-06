# -*- coding: utf-8 -*-
"""
Created on Tue May 11 15:49:03 2021

@author: I3D-MSI
"""
import numpy as np
import cv2, os
import matplotlib.image as mpimg
from scipy.misc import imresize
# from moviepy.editor import VideoFileClip
from IPython.display import HTML
# import tensorflow.keras as keras
# from keras.models import model_from_json
from tensorflow.keras.models import model_from_json
from tensorflow.keras.models import load_model
from scipy.optimize import curve_fit
import matplotlib.pyplot as plt
from glob import glob
from time import time
import csv
import socket

# load model
json_file = open(r"json_file_39.json", 'r')
json_model = json_file.read()
json_file.close()
model = model_from_json(json_model)
model.load_weights(r"best_weights_39.h5")
connect = False

with open('lane_log.csv', 'w') as f:
    csv_f = csv.writer(f)
    csv_f.writerow([0, 0, 0])
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

def func(x, a, b, c):
    return a*x**2 + b*x + c

def getCentroid(mask):
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.erode(mask, kernel, iterations=1)
    mask = cv2.dilate(mask, kernel, iterations=2)
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    x = 0
    y = 0
    if len(contours) > 0:
        contours = sorted(contours, key=cv2.contourArea, reverse=True)
        contours = np.reshape(contours[0], (contours[0].shape[0], 2))
        x = np.mean(contours[:, 0])
        y = np.mean(contours[:, 1])
    return x, y
def getMasks(frame):
    hsvFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lowGreen = np.array([40, 100, 50])  # np.array([79, 120, 80])
    highGreen = np.array([80, 255, 255])  # np.array([82, 250, 180])
    # lowRed1 = np.array([0, 100, 20])
    # highRed1 = np.array([10, 255, 255])
    # lowRed2 = np.array([160, 100, 20])  # np.array([161, 155, 85])
    # highRed2 = np.array([180, 255, 255])  # np.array([179, 255, 255])

    # lowRedMask = cv2.inRange(hsvFrame, lowRed1, highRed1)
    # upRedMask = cv2.inRange(hsvFrame, lowRed2, highRed2)
    # redMask = lowRedMask + upRedMask
    greenMask = cv2.inRange(hsvFrame, lowGreen, highGreen)
    gx, gy = getCentroid(greenMask)
    return gx, gy
dir = 0
def get_midlane(img, img2, rect_width=220, rect_height=200, rect_ypos=0, lw=20, st_threshold=0, pixel=4):
    global dir
    x, y = getMasks(img2)
    if x == 0 and y == 0:
        print("None")
    elif x < img2.shape[1]/2:
        dir = 1
    else:
        dir = 0
    kernel = np.ones((pixel+2,pixel+2))
    img = cv2.filter2D(img, -1, kernel)
    img_shape = img.shape[:2]
    rect_xpos = int((img_shape[0] - rect_width) / 2)
    centredot_ypos = int(rect_ypos + rect_height / 2)
    mask = np.zeros(img_shape, dtype="uint8")
    cv2.rectangle(mask, (rect_xpos, rect_ypos), (rect_xpos + rect_width, rect_ypos + rect_height), 255, -1)
    img = cv2.bitwise_and(img, img, mask=mask)
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    contours, hierarchy = cv2.findContours(img_gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    contours_no = len(contours)


    left_mask = np.zeros(img_shape, dtype="uint8")
    right_mask = np.zeros(img_shape, dtype="uint8")
    half_x = int(img_shape[1] / 2)
    cv2.rectangle(left_mask, (0, 0), (half_x, img_shape[0] - 1), 255, -1)
    cv2.rectangle(right_mask, (half_x, 0), (img_shape[1] - 1, img_shape[0] - 1), 255, -1)
    left_mask = cv2.bitwise_and(img_gray, img_gray, mask=left_mask)
    right_mask = cv2.bitwise_and(img_gray, img_gray, mask=right_mask)
    # cv2.imshow("left", left_mask)
    contours1, hierarchy1 = cv2.findContours(left_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    contours1 = sorted(contours1, key=cv2.contourArea, reverse=True)
    contours2, hierarchy1 = cv2.findContours(right_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    contours2 = sorted(contours2, key=cv2.contourArea, reverse=True)
    left_contours = len(contours1)
    right_contours = len(contours2)
    if (len(contours)) == 0:
        return img, "None", contours_no, left_contours, right_contours
    if (len(contours1) == 0):
        return img, "None Left", contours_no, left_contours, right_contours
    elif (len(contours2) == 0):
        return img, "None Right", contours_no, left_contours, right_contours
    lst = [0] * 2

    lst[0] = np.reshape(contours1[0], (contours1[0].shape[0], 2))
    lst[1] = np.reshape(contours2[0], (contours2[0].shape[0], 2))

    x1 = lst[0][:,0]
    y1 = lst[0][:,1]
    x2 = lst[1][:,0]
    y2 = lst[1][:,1]
    try:
        popt1, pcov = curve_fit(func, y1, x1)
        popt2, pcov = curve_fit(func, y2, x2)
    except:
        return img, "None", contours_no, left_contours, right_contours
    y_fit = 50
    x1_line = func(y_fit, *popt1)
    x2_line = func(y_fit, *popt2)
    x3 = (x1_line + x2_line) / 2
    x3 = x3.astype(int)
    p1 = (x3, y_fit + int(lw/2))
    p2 = (x3, y_fit - int(lw/2))
    rect_centre = (int(rect_xpos + rect_width / 2), int(rect_ypos + rect_height / 2))
    cv2.line(img, p1, p2, (255, 255, 255))
    cv2.circle(img, rect_centre, radius=1, color=(0, 0, 255), thickness=1)
    cv2.rectangle(img, (rect_xpos, rect_ypos), (rect_xpos + rect_width, rect_ypos + rect_height), 255, 1)
    if((rect_centre[0] - st_threshold) <= x3 <= (rect_centre[0] + st_threshold)):
        control = 0
    elif x3 < rect_centre[0]:
        control = rect_centre[0] - x3
    else:
        control = rect_centre[0] - x3

    if min(left_contours, right_contours) > 4:
        control = "Take Request" + str(dir)

    return img, control, contours_no, left_contours, right_contours



def start(threshold=20):
    cap = cv2.VideoCapture(0)
    cap.set(3, 1920)
    cap.set(4, 1080)
    frame_width = (int)(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = (int)(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = (int)(cap.get(cv2.CAP_PROP_FPS))
    print(fps)
    # out = cv2.VideoWriter("output_realtime_2.avi", cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), 25, (1280, 720))
    while (True):
        ret, frame = cap.read()
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        # colorMask = getMasks(frame)
        img = cv2.resize(frame, (1280, 720))
        input_height = 224  # 80
        input_width = 224  # 160
        small_img_2 = imresize(img, (input_height, input_width, 3))
        small_img_1 = np.array(small_img_2)
        small_img = small_img_1[None, :, :, :]
        prediction = model.predict(small_img)[0] * 255
        prediction = prediction.astype(np.uint8)
        prediction2 = np.copy(prediction)
        prediction2 = cv2.threshold(prediction2, 40, 255, cv2.THRESH_BINARY)[1]
        width = img.shape[1]
        height = img.shape[0]
        blanks = np.zeros_like(prediction2).astype(np.uint8)
        lane_drawn = np.dstack((blanks, prediction2, blanks))
        lane_drawn, control, contours_no, left_contours, right_contours = get_midlane(lane_drawn, small_img_2)
        with open('lane_log.csv', 'a') as f:
            csv_f = csv.writer(f)
            csv_f.writerow([left_contours, right_contours, control])
        if len(lane_drawn) == 0:
            continue
        if connect:
            sock.sendto(str(control).encode('utf-8'), serverAddressPort)
        lane_image = imresize(lane_drawn, (height, width, 3))
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(lane_image, 'Total: ' + str(contours_no), (int((width / 2) - 500), int((height / 2) - 10)), font, 2, (255, 255, 255), 4,
                    cv2.LINE_AA)
        cv2.putText(lane_image, 'Left: ' + str(left_contours), (int((width / 2) - 500), int((height / 2) - 200)), font, 2,
                    (255, 255, 255), 4,
                    cv2.LINE_AA)
        cv2.putText(lane_image, 'Right:' + str(right_contours), (int((width / 2) + 250), int((height / 2) - 200)), font, 2,
                    (255, 255, 255), 4,
                    cv2.LINE_AA)
        cv2.putText(lane_image, 'Control:' + str(control), (int((width / 2) + 250), int((height / 2) + 200)), font,
                    2,
                    (255, 255, 255), 4,
                    cv2.LINE_AA)
        result = cv2.addWeighted(img, 0.7, lane_image, 1, 0)
        result1 = cv2.resize(result, (1920, 1080))
        # blanks = np.zeros_like(colorMask).astype(np.uint8)
        # colorMask = np.dstack((blanks, colorMask, blanks))
        # result1 = cv2.addWeighted(result1, 0.7, colorMask, 1, 0)
        cv2.imshow('frame', result1)
        # out.write(result)
    cap.release()
    cv2.destroyAllWindows()


start()
