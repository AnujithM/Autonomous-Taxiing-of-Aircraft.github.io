#!/usr/bin/python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray
import pickle
print("Inside wheel control")
state = 1
def getAngularVelocity(control):
    return 2 - control/64

def getLinearVelocity(control):
    if control >= 128:
        return 0.0
    else:
        return 0.2 - control/854

def getState(control):
    global state
    if control == 0:
        pass
    elif control == 8:
        state = -1
    else:
        state = 1
def controlCallback(data, args):
    global state
    pub = args[0]
    twist = args[1]
    control = data.data
    print(control)
    getState(control[10])
    twist.linear.x = state*getLinearVelocity(control[9])
    # if control[9] < 128:
    #     twist.linear.x = 1.0
    # else:
    #     twist.linear.x = 0.0

    twist.angular.z = getAngularVelocity(control[1])
    # if control[1] == 128:
    #     twist.angular.z = 0.0
    # elif control[1] < 128:
    #     twist.angular.z = 1.0
    # else:
    #     twist.angular.z = -1.0
    pub.publish(twist)

if __name__=="__main__":
    rospy.init_node('turtlebot3_wheel_control')
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    turtlebot3_model = rospy.get_param("model", "burger")
    twist = Twist()
    twist.linear.y = 0.0
    twist.linear.z = 0.0
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    rospy.Subscriber('control_info', Int32MultiArray, controlCallback, (pub, twist))
    rospy.spin()

    # while True:
    #     with open("control", "r") as f:
    #         control = f.read()
    #     print(control)
    #     control = pickle.loads(control)
    #     angular_control = control[1]
    #     linear_control = control[9]
    #     if linear_control > 128:
    #         twist.linear.x = 1.0
    #     else:
    #         twist.linear.x = 0
    #     pub.publish(twist)
