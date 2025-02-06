#! /usr/bin/env python

import roslib
import rospy
import tf.transformations
from geometry_msgs.msg import Twist, PolygonStamped
import socket
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from visualization_msgs.msg import Marker 
from visualization_msgs.msg import MarkerArray 
from geometry_msgs.msg import Point 
import time
import keyboard
from threading import Thread
import numpy as np
import math
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from getkey import getkey

x = 0.0
y = 0.0
theta = 0.0
err = 0.065

topic = 'visualization_marker_array' 
publisher = rospy.Publisher(topic, MarkerArray, queue_size=5)
markerArray = MarkerArray()
count = 0

Ip = "192.168.43.197"
port = 20001
UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
UDPServerSocket.bind((Ip, port))
data1,addr1 = UDPServerSocket.recvfrom(1024)   #This is communication from Hololens
print(data1.decode('utf-8'))
data,addr = UDPServerSocket.recvfrom(1024)     #This is communication from dobot1
print(data.decode('utf-8'))
UDPServerSocket.sendto("Hi Client1".encode('utf-8'),addr)
data_,addr_ = UDPServerSocket.recvfrom(1024)     #This is communication from dobot2
print(data_.decode('utf-8'))
UDPServerSocket.sendto("Hi Client2".encode('utf-8'),addr_)
UDPServerSocket.settimeout(1.5)

def move():
    # Starts a new node
    #rospy.init_node('more_dist', anonymous=True)
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()
    #print("Moved into the workspace")
    distance = 0.12
    speed = 0.2
    vel_msg.linear.x = 0.2
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0
    t0 = rospy.Time.now().to_sec()
    current_distance = 0
    while(current_distance < distance):
        velocity_publisher.publish(vel_msg)
        t1=rospy.Time.now().to_sec()
        current_distance= speed*(t1-t0)
        #print(current_distance)
        time.sleep(0.1)
    vel_msg.linear.x = 0
    print("Done")
    velocity_publisher.publish(vel_msg)

def newOdom (msg):
    global x
    global y
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
    x = (ans[0]) / (6 * signedArea)
    y = (ans[1]) / (6 * signedArea)
    #print(x,y)
    return

def movebase_client(x_,y_,th_):
    global client
    print("Goal sent")
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x_
    goal.target_pose.pose.position.y = y_
    q = quaternion_from_euler(0, 0, th_)
    goal.target_pose.pose.orientation.x = q[0]
    goal.target_pose.pose.orientation.y = q[1]
    goal.target_pose.pose.orientation.z = q[2]
    goal.target_pose.pose.orientation.w = q[3]
    print(q)
    # goal.target_pose.pose.orientation.z = th_
    # goal.target_pose.pose.orientation.w = 1.0

    client.send_goal(goal)
        # wait = client.wait_for_result()
        # if not wait:
        #     rospy.logerr("not available")
        #     rospy.signal_shutdown("not available")
        # else:
        #     return client.get_result()

def move_to(x,y,th):
    result = movebase_client(x,y,th)
    #try:
        # rospy.init_node('movebase_client')
        # global UDPServerSocket
        # print("Here")
        # data,addr = UDPServerSocket.recvfrom(1024)
        # print("Here")
        # print(data.decode('utf-8'))
        # A = str(x)+" "+str(y)
        # message = A.encode('utf-8')
        # UDPServerSocket.sendto(message, addr)
        #result = movebase_client(x,y,th)
        #if result:
            #move = Twist()
            #rospy.loginfo("Goal execution done!")
            
            #UDPServerSocket.bind((Ip, port))
            # data,addr = UDPServerSocket.recvfrom(1024)
            # print(data.decode('utf-8'))
            # message = "Pick the object".encode('utf-8')
            # UDPServerSocket.sendto(message, addr)
            #exit()
        #else:
            #msg = "Fail"
            # UDPServerSocket.bind((Ip, port))
            # data,addr = UDPServerSocket.recvfrom(1024)
            # print(data.decode('utf-8'))
            # message = msg.encode('utf-8')
            # UDPServerSocket.sendto(message, addr)
            #exit()
    #except rospy.ROSInterruptException:
        #rospy.loginfo("Navigation done")




sub = rospy.Subscriber('/move_base/global_costmap/footprint', PolygonStamped, newOdom)


rospy.init_node('movebase_client')
db_X = -2.63  #-1.1 #6.05#6.15 #6.32 #2.5 
db_Y = -1.12 #0.0 #-1.57 #-1.49 #-1.22 #-0.81 
db_th = math.pi/2

#db_X = 0
#db_Y = 0
#db_th = 0


l = np.arange(-90,90,3)
l = np.append(l,90)
xl = np.zeros(len(l))
yl = np.zeros(len(l))

#print(l)
for i in range(0,len(l)):
    xl[i] = db_X+0.45*(math.cos(db_th+(math.pi*l[i]/180)))
    yl[i] = db_Y+0.45*(math.sin(db_th+(math.pi*l[i]/180)))
    marker = Marker()
    marker.id = count
    marker.lifetime = rospy.Duration()
    marker.header.frame_id = "map"
    marker.type = marker.SPHERE
    marker.action = marker.ADD
    marker.scale.x = 0.03
    marker.scale.y = 0.03
    marker.scale.z = 0.03
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = xl[i] 
    marker.pose.position.y = yl[i] 
    marker.pose.position.z = 0 
    markerArray.markers.append(marker) 
    count += 1
    #print(xl[i],yl[i],"were probable points")
print("waiting 2 seconds before displaying markers")
rospy.sleep(2)
publisher.publish(markerArray)
dist = np.zeros(len(l))
current_dist = 0

current = 1
chosen = 1


def change_dobot():
    global db_X,db_Y,db_th,l,xl,yl,dist,current_dist,count,current,chosen
    if chosen==1:
        db_X = -2.63 #-1.0 #6.13 #-1.0 #6.17 #6.32 #2.5 
        db_Y = -1.12 #0.0 #-1.48 #0.0 #-1.52 #-1.22 #-0.81 
        db_th = math.pi/2 #0 #math.pi/2
    if chosen==2:
        db_X = -3.64  #1.1 #7.34 #1.1 #6.17
        db_Y = -0.432 #0.0 #0.048 #0.5
        db_th = -1*math.pi #0 #-1*math.pi/2

    #db_X = 0
    #db_Y = 0
    #db_th = 0


    l = np.arange(-90,90,3)
    l = np.append(l,90)
    xl = np.zeros(len(l))
    yl = np.zeros(len(l))

    rad = 0.5#0.45 was original
    #print(l)
    for i in range(0,len(l)):
        xl[i] = db_X+rad*(math.cos(db_th+(math.pi*l[i]/180)))
        yl[i] = db_Y+rad*(math.sin(db_th+(math.pi*l[i]/180)))
        marker = Marker()
        marker.id = count
        marker.lifetime = rospy.Duration()
        marker.header.frame_id = "map"
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = 0.03
        marker.scale.y = 0.03
        marker.scale.z = 0.03
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = xl[i] 
        marker.pose.position.y = yl[i] 
        marker.pose.position.z = 0 
        markerArray.markers.append(marker) 
        count += 1
        #print(xl[i],yl[i],"were probable points")
    print("waiting 2 seconds before displaying markers")

def wait_time():
    #print("Waiting")
    time.sleep(2)

def keyBinput():
    #print("Inside keyBinput")
    global chosen,UDPServerSocket,addr1
    tStart = time.time()
    try:
        data,addr1 = UDPServerSocket.recvfrom(1096)
        if data.decode('utf-8')=="1":
            print("1 was received")
            chosen = 1
            UDPServerSocket.sendto("stop 2".encode('utf-8'),addr)
        if data.decode('utf-8')=="2":
            print("2 was received")
            chosen = 2
            UDPServerSocket.sendto("stop 1".encode('utf-8'),addr)
    except:
        print("No input")
    # while time.time()-tStart<=2:
    #     key = getkey()
    #     chosen = int(key)
    #     print(key,"was pressed",chosen,"was updated")
    print("Outside keyBinput",chosen)
        

dist = np.zeros(len(l))
current_dist = 0

current = 1
chosen = 1
#while not rospy.is_shutdown():
tStart = time.time()
while True:
    print("Current dobot",current)

    if chosen!=current:
        print("new dobot chosen",chosen)
        current=chosen
        change_dobot()
    #print("current position is: ",x, y)
    for i in range(len(l)):
        dist[i] = math.sqrt(pow(xl[i]-x,2)+pow(yl[i]-y,2))
        #dist[i] = math.sqrt(pow((xl[i]+0.25)-(x+0.25),2)+pow((yl[i]+0.65)-(y+0.65),2))
    #print(dist)
    ind = np.argmin(dist)
    xg = xl[ind]
    #print("Xg is: ",xg)
    yg = yl[ind]
    #print("Yg is: ")
    xg_ = xg*math.cos(err) + yg*math.sin(err)
    yg_ = yg*math.cos(err) - xg*math.sin(err)
    #print("New goal Xg is: ", xg_)
    #print("New goal Yg is: ", yg_)
    if chosen == 1:
        UDPServerSocket.sendto(str(l[ind]).encode('utf-8'),addr)
    else:
        UDPServerSocket.sendto(str(l[ind]).encode('utf-8'),addr_)

    if xg-db_X > 0:
        #print("greater than 0")
        thg = math.atan2(yg-(db_Y),xg-(db_X)) + math.pi
    else:
        #print("Less than zero")
        thg = math.atan2(yg-(db_Y),xg-(db_X)) - math.pi
    # if yg-db_Y>0:
    #   thg = math.atan2(yg-(db_Y),xg-(db_X)) - math.pi
    # else:
    #   thg = math.atan2(yg-(db_Y),xg-(db_X))
    #print(xg,yg,thg,"was goal position")
    move_to(xg,yg,thg)
    current_dist = math.sqrt(pow(db_X-x,2)+pow(db_Y-y,2))
    #print("distance from dobot: ", current_dist)
    if current_dist < 0.8: #1.9 for normal
        print("Tolerance reached")
        wait = client.wait_for_result()
        if chosen==1:
            UDPServerSocket.sendto("End 2".encode('utf-8'),addr)
            UDPServerSocket.sendto("End 2".encode('utf-8'),addr_)
        if chosen==2:
            UDPServerSocket.sendto("End 1".encode('utf-8'),addr)
            UDPServerSocket.sendto("End 1".encode('utf-8'),addr_)
        if not wait:
            rospy.logerr("not available")
            rospy.signal_shutdown("not available")
        print("Exiting loop")
        break
    #Thread(target = wait_time).start()
    #Thread(target = keyBinput).start()
    keyBinput()
    #time.sleep(2)
    #rospy.spin()
print("x and y is:", x, y)
print("moving straight")
move()
if chosen==1:
    UDPServerSocket.sendto("Pick 1".encode('utf-8'),addr)
    UDPServerSocket.sendto("Pick 1".encode('utf-8'),addr_)
else:
    UDPServerSocket.sendto("Pick 2".encode('utf-8'),addr)
    UDPServerSocket.sendto("Pick 2".encode('utf-8'),addr_)
print("exiting script")
#exit()
