#! /usr/bin/env python

import roslib
import rospy
import tf.transformations
from geometry_msgs.msg import Twist
import socket
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import time
import numpy as np
import math
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

x = 0.0
y = 0.0
theta = 0.0
err = 0.065

# Ip = "192.168.1.116"
# port = 20001
# UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
# UDPServerSocket.bind((Ip, port))

def move():
    # Starts a new node
    #rospy.init_node('more_dist', anonymous=True)
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()
    print("Moved into the workspace")
    distance = 0.375
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
    	print(current_distance)
    	time.sleep(0.1)
    vel_msg.linear.x = 0
    print("Velocity zero")
    velocity_publisher.publish(vel_msg)

def newOdom (msg):
	global x
	global y
	global theta
	x = msg.pose.pose.position.x
	y = msg.pose.pose.position.y
	rot_q = msg.pose.pose.orientation
	(roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
	return

def movebase_client(x_,y_,th_):

    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x_
    goal.target_pose.pose.position.y = y_

    goal.target_pose.pose.orientation.z = th_
    goal.target_pose.pose.orientation.w = 1.0

    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("not available")
        rospy.signal_shutdown("not available")
    else:
        return client.get_result()

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



sub = rospy.Subscriber('/odom', Odometry, newOdom)


rospy.init_node('movebase_client')
db_X = 6.1440348625183105 #2.5 
db_Y = -1.3808107376098633 #-0.81 
db_th = math.pi/2

#db_X = 0
#db_Y = 0
#db_th = 0


l = np.arange(-90,90,3)
l = np.append(l,90)
xl = np.zeros(len(l))
yl = np.zeros(len(l))

print(l)
for i in range(0,len(l)):
	xl[i] = db_X+0.7*(math.cos(db_th+(math.pi*l[i]/180)))
	yl[i] = db_Y+0.7*(math.sin(db_th+(math.pi*l[i]/180)))
	print(xl[i],yl[i],"were probable points")

dist = np.zeros(len(l))


#while not rospy.is_shutdown():
for i in range(len(l)):
	print(x, y)
	dist[i] = math.sqrt(pow(xl[i]-x,2)+pow(yl[i]-y,2))
	#dist[i] = math.sqrt(pow((xl[i]+0.25)-(x+0.25),2)+pow((yl[i]+0.65)-(y+0.65),2))
print(dist)
ind = np.argmin(dist)
xg = xl[ind]
print("Xg is: ",xg)
yg = yl[ind]
print("Yg is: ")
xg_ = xg*math.cos(err) + yg*math.sin(err)
yg_ = yg*math.cos(err) - xg*math.sin(err)
print("New goal Xg is: ", xg_)
print("New goal Yg is: ", yg_)

if yg-db_Y>0:
	thg = math.atan2(yg-(db_Y),xg-(db_X)) - math.pi + err
else:
	thg = math.atan2(yg-(db_Y),xg-(db_X)) + math.pi  err
print(xg,yg,thg,"was goal position")
move_to(xg,yg,thg)
move()
exit()
	#rospy.spin()
