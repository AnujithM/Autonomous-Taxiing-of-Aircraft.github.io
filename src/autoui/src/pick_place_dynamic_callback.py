#! /usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import time
import numpy as np
import math
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist, Point, Quaternion


#rospy.init_node('movebase_client')

db_X = 6.1
db_Y = -1.4
db_th = math.pi/2


x_ = 0
y_ = 0
theta = 0


l = np.arange(-90,90,3)
l = np.append(l,90)
xl = np.zeros(len(l))
yl = np.zeros(len(l))
#print(l)

for i in range(0,len(l)):
    xl[i] = db_X+0.6*(math.cos(math.pi*l[i]/180))
    yl[i] = db_Y+0.6*(math.sin(math.pi*l[i]/180))
    #print(xl[i],yl[i],"were probable points")

dist = np.zeros(len(l))

def callback(msg):



	global x_curr
	global y_curr
	current_dist = 0.0

	x_curr = msg.pose.pose.position.x
	y_curr = msg.pose.pose.position.y
	print(x_curr, y_curr)

	for i in range(len(l)):
		dist[i] = math.sqrt(pow(xl[i]-x_curr,2)+pow(yl[i]-y_curr,2))
	#print(dist)
	ind = np.argmin(dist)
	xg = xl[ind]
	yg = yl[ind]
	if yg-db_Y > 0:
		thg = math.atan2(yg-db_Y,xg-db_X) - math.pi
	else:
		thg = math.atan2(yg-db_Y,xg-db_X) + math.pi
	print(xg,yg, thg, "is new goal position")
	


	client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
	client.wait_for_server()
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = "map"
	goal.target_pose.header.stamp = rospy.Time.now()
	goal.target_pose.pose.position.x = xg
	goal.target_pose.pose.position.y = yg
	goal.target_pose.pose.orientation.z = thg
	client.send_goal(goal)
	time.sleep(3)
	#sock.sendto(str(l[ind]).encode('utf-8'),addr)
	#time.sleep(3)
	curr_dist = math.sqrt(pow(db_X-x_curr,2)+pow(db_Y-y_curr,2))
	if current_dist < 0.7:
		exit()




rospy.init_node('check_odometry')
odom_sub = rospy.Subscriber('/odom', Odometry, callback)
rospy.spin()
