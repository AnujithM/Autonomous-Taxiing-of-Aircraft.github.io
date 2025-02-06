#! /usr/bin/env python

from ctypes import Array
import rospy
import roslib
from pickle import NONE
from flask import Flask, render_template, send_from_directory, Response, request, jsonify
# from flask_socketio import SocketIO
from pathlib import Path
import tf.transformations
from capture import capture_and_save
from camera import Camera
import argparse, logging, logging.config, conf
import csv, json
import actionlib
import socket

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
#from sklearn import linear_model
import time

#rospy.init_node('movebase_server')

with open('prev_flag.txt','w') as prev:
	prev.write("1")
	print("Wrote 1 to pre")

# with open('current_flag.txt','w') as prev:
# 	prev.write("1")
# 	print("Wrote 1 to pre")

localIP     = "192.168.43.197"

localPort   = 20001

address = (localIP,localPort)

msgFromServer       = "Hello UDP Intermediate Client"
byte                = str.encode(msgFromServer)
UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
UDPServerSocket.sendto(byte, address)


logging.config.dictConfig(conf.dictConfig)
logger = logging.getLogger(__name__)

camera = Camera()
camera.run()
prev_flag = "1"
stop_flag = "0"
file1 = "Name_Spawn_Start.txt"
file2 = "Name_Spawn_End.txt"

app = Flask(__name__)
# app.config["SECRET_KEY"] = "secret!"
# socketio = SocketIO(app)


@app.after_request
def add_header(r):
	"""
	Add headers to both force latest IE rendering or Chrome Frame,
	and also to cache the rendered page for 10 minutes
	"""
	r.headers["Cache-Control"] = "no-cache, no-store, must-revalidate"
	r.headers["Pragma"] = "no-cache"
	r.headers["Expires"] = "0"
	r.headers["Cache-Control"] = "public, max-age=0"
	return r

@app.route("/")
def entrypoint():
	logger.debug("Requested /")
	return render_template("index.html")

@app.route("/r")
def capture():
	logger.debug("Requested capture")
	im = camera.get_frame(_bytes=False)
	capture_and_save(im)
	return render_template("send_to_init.html")

@app.route("/images/last")
def last_image():
	logger.debug("Requested last image")
	p = Path("images/last.png")
	if p.exists():
		r = "last.png"
	else:
		logger.debug("No last image")
		r = "not_found.jpeg"
	return send_from_directory("images",r)


def gen(camera):
	logger.debug("Starting stream")
	while True:
		frame = camera.get_frame()
		yield (b'--frame\r\n'
			   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route("/stream")
def stream_page():
	logger.debug("Requested stream page")
	return render_template("stream.html")

@app.route("/video_feed")
def video_feed():
	return Response(gen(camera),
		mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/_array2python", methods=['POST'])
def array2python():
    my_ar = int(request.get_data())
	


@app.route("/api", methods=['GET','POST'])
def clicked():
	global UDPServerSocket
	global address
	global prev_flag
	global stop_flag
	#direction = int(request.get_data())
	coordinates = request.get_data().decode('utf-8').split(',')
	# client_id = request.args.get('client_id')
	#x = float(coordinates[2]) * 1000 + 89.1383
	#y = -float(coordinates[0]) * 1000
	#z = -float(coordinates[1]) * 1000 - 0.0016
	s="hello"
	# x_ = float(coordinates[2]) * 1000 
	# y_ = -float(coordinates[0]) * 1000
	# z_ = -float(coordinates[1]) * 1000
	# x = x_/1000 
	# y = -1*z_/1000
	# print('Captured Click', x , "," , y )
	str_ = request.get_data().decode('utf-8')
	print(request.get_data().decode('utf-8'),"is full")
	# XY = str(x)+" "+str(y)
	# XYb = XY.encode('utf-8')
	if str_=="1":
		UDPServerSocket.sendto(request.get_data(),address)  #Uncomment for regular pick place only
		return s

	if str_ == "10":#Spawn Start
		with open(file1,'w') as f1:
			f1.write(str(time.time()))
	if str_ == "11":
		with open(file2,'w') as f2:
			f2.write(str(time.time()))
	if str_ == '8': #Stop
		with open("Stop.txt",'w') as st:
			st.write("1")
	if str_ == '9':
		with open("Stop.txt",'w') as st:
			st.write("0")
	if str_ != '7':
		with open('Auto.txt','w') as f:
			f.write(str_)
	else:
		print(prev_flag,"here")
		if prev_flag == "1":
			flag = "0"
			with open('current_flag.txt','w') as curr:
				curr.write(flag)
				print("Wrote",flag,"to curr","prev was",prev_flag)
			prev_flag = "0"
		else:
			flag = "1"
			with open('current_flag.txt','w') as curr:
				curr.write(flag)
				print("Wrote",flag,"to curr","prev was",prev_flag)
			prev_flag = "1"
	# Uncomment above
	#movebase_client(x,y,0)
	# rospy.init_node('movebase_client')
	# client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
	# client.wait_for_server()

	# goal = MoveBaseGoal()
	# goal.target_pose.header.frame_id = "map"
	# goal.target_pose.header.stamp = rospy.Time.now()
	# goal.target_pose.pose.position.x = x
	# goal.target_pose.pose.position.y = y

	# goal.target_pose.pose.orientation.z = 0.0
	# goal.target_pose.pose.orientation.w = 1.0

	# client.send_goal(goal)
	# wait = client.wait_for_result()
	return s



if __name__=="__main__":
	# socketio.run(app,host="0.0.0.0",port="3005",threaded=True)
	parser = argparse.ArgumentParser()
	parser.add_argument('-p','--port',type=int,default=5000, help="Running port")
	parser.add_argument("-H","--host",type=str,default='0.0.0.0', help="Address to broadcast")
	args = parser.parse_args()
	logger.debug("Starting server")
	app.run(host=args.host,port=args.port)
