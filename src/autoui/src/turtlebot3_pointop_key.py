#!/usr/bin/env python
#################################################################################
# Copyright 2018 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#################################################################################

# Authors: Gilbert #

import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
import tf
from math import radians, copysign, sqrt, pow, pi, atan2
from tf.transformations import euler_from_quaternion
import numpy as np
import time
import socket


msg = """
control your Turtlebot3!
-----------------------
Insert xyz - coordinate.
x : position x (m)
y : position y (m)
z : orientation z (degree: -180 ~ 180)
If you want to close, insert 's'
-----------------------
"""

cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
d = 1
dp = 1
Ip = "192.168.0.16"
port = 20001
UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
UDPServerSocket.bind((Ip, port))
data_,addr = UDPServerSocket.recvfrom(1024)  #Communication from lane detection file
print(data_.decode('utf-8'))
UDPServerSocket.sendto("Hi Client".encode('utf-8'),addr)




Kp = 0.005
Kd = 0.01

def steer():
    global cmd_vel
    global d
    global dp
    moveL = Twist()
    print(d,"was received")
    moveL.linear.x = -0.08
    if d>0:
        if d*Kp + (d-dp)*Kd>0.06:
            moveL.angular.z = 0.06
        else:
            moveL.angular.z = d*Kp
    else:
        if d*Kp + (d-dp)*Kd<-0.06:
            moveL.angular.z = -0.06
        else:
            moveL.angular.z = d*Kp
    cmd_vel.publish(moveL)
    dp = d
    time.sleep(0.1)

def stop_tb():
    move = Twist()
    move.linear.x = 0.0
    move.linear.y = 0.0
    move.linear.z = 0.0
    move.angular.x = 0.0
    move.angular.y = 0.0
    move.angular.z = 0.0
    cmd_vel.publish(move)

def steer_stop(con):
#    global UDPServerSocket
    global d
    moveL = Twist()
    moveL.linear.x = 0
    moveL.angular.z = 0
    cmd_vel.publish(moveL)
    if con == "None Right":
        print("Left over steer")
        moveL.linear.x = -0.05
        moveL.angular.z = -0.04
        cmd_vel.publish(moveL)
    else:
        print("Right over steer")
        moveL.linear.x = -0.05
        moveL.angular.z = 0.04
        cmd_vel.publish(moveL)
    # data,addr = UDPServerSocket.recvfrom(1024)
    # if data_.decode('utf-8')=="None Left" or data_.decode('utf-8')=="None Right":
    #     time.sleep(0.1)
    #     steer_stop(data_.decode('utf-8'))
    # else:
    #     print(data.decode('utf-8'),"is received")
    #     time.sleep(0.1)
    #     return
    time.sleep(0.1)




rospy.init_node('Lane_Detection')
X = time.time()

count = 0;
count2 =0;
check2 = 'dg';
with open('stop.txt','w') as f:
    f.write('1')

with open('current_flag.txt','w') as cu:
    cu.write('1')



while True:
    data_,addr_ = UDPServerSocket.recvfrom(1024)
    if addr_ == addr:
        with open('Lane_Detection.txt','w') as lane:
            lane.write(data_.decode('utf-8'))
        print(data_.decode('utf-8'),"was received")
    # with open('stop.txt','r') as c:
    #     check2 = c.read()
    # with open('current_flag.txt','r') as flag:
    #     check = flag.read()
    #     if check == '0' or check2 == '0':
    #         # if check == '1':
    #         #     count = 0
    #         #     count2 = 0
    #         continue

    #     else:
    #         data_,addr_ = UDPServerSocket.recvfrom(1024)
    #         if addr_ == addr:
    #             with open('Lane_Detection.txt','w') as lane:
    #                 lane.write(data_.decode('utf-8'))
            # if data_.decode('utf-8')=="None":
            #     print("End of the road")
            #     count2 = count2+1
            #     if count2>3:
            #         with open('stop.txt','w') as stop:
            #             stop.write('0')
            #     stop_tb()
            #     continue
            # if data_.decode('utf-8')=="Take Request":
            #     count = count+1
            #     if count>3:
            #         with open('stop.txt','w') as stop:
            #             stop.write('0')
            #     stop_tb()
            #     continue
            # if data_.decode('utf-8')=="None Left" or data_.decode('utf-8')=="None Right":
            #     print(data_.decode('utf-8'))
            #     steer_stop(data_.decode('utf-8'))
            # elif addr_ == addr:
            #     d = int(data_.decode('utf-8'))
            #     steer()
