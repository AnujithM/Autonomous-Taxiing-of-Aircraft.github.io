#! /usr/bin/env python

import roslib
import rospy
import tf.transformations
from geometry_msgs.msg import Twist
import socket
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import time


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
	try:
		#rospy.init_node('movebase_client')
		result = movebase_client(x,y,th)
		if result:
			rospy.loginfo("Goal execution done!")
	except rospy.ROSInterruptException:
		rospy.loginfo("Navigation done")


localIP     = "192.168.1.118"

localPort   = 2401

msg = "Hello Server"

#rospy.init_node('speed_control')
rospy.init_node('movebase_client')
pub = rospy.Publisher('/cmd_vel',Twist,queue_size=1)
move = Twist()

print(msg)

client_socket = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
#rate = rospy.Rate(10000000)

"""
while not rospy.is_shutdown():
	client_socket.sendto(msg.encode("utf-8"),(localIP,localPort))
	data,addr = client_socket.recvfrom(8192)
	#print(data.decode('utf-8'))
	V_ = ""
	W_ = ""
	check = 0
	str_ = data.decode('utf-8')
	List = data.split()
	
	if len(List)==2:
		V = float(List[0])
		W = float(List[1])
		print(V,W,"were received")
		#print(V,W,"were received")
		move.linear.x = V
		move.angular.z = W
		pub.publish(move)
		#rate.sleep()
	else:
		x = float(List[0])
		y = float(List[1])
		th = float(List[2])
		print("Moving to ",x,y,th)
		move_to(x,y,th)
"""


while not rospy.is_shutdown():
	client_socket.sendto(msg.encode("utf-8"),(localIP,localPort))
	data,addr = client_socket.recvfrom(8192)
	#print(data.decode('utf-8'))
	V_ = ""
	W_ = ""
	check = 0
	str_ = data.decode('utf-8')
	List = data.split()
	
	if len(List)==2:
		V = float(List[0])
		W = float(List[1])
		print(V,W,"were received")
		#print(V,W,"were received")
		move.linear.x = V
		move.angular.z = W
		pub.publish(move)
		#rate.sleep()
	else:
		x = float(List[0])
		y = float(List[1])
		th = float(List[2])
		print("Moving to ",x,y,th)
		move_to(x,y,th)


client_socket.close()
