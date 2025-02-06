#! /usr/bin/env python

import rospy
import actionlib
import math
from tf.transformations import quaternion_from_euler
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def movebase_client():

    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = 0
    goal.target_pose.pose.position.y = 0
    q = quaternion_from_euler(0, 0, 0)
    goal.target_pose.pose.orientation.x = q[0]
    goal.target_pose.pose.orientation.y = q[1]
    goal.target_pose.pose.orientation.z = q[2]
    goal.target_pose.pose.orientation.w = q[3]
    print(q)

    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("not available")
        rospy.signal_shutdown("not available")
    else:
        return client.get_result()


try:
    rospy.init_node('movebase_client')
    result = movebase_client()
    if result:
        rospy.loginfo("Goal execution done!")
except rospy.ROSInterruptException:
    rospy.loginfo("Navigation done")
    
    
    
   #x: -2.5204879420925863e-05
      #y: -2.3117668206396047e-06

      
      
      #x: -2.5204879420925863e-05
      #y: -2.3117668206396047e-06
      
    #goal.target_pose.pose.position.x = 5.0163290188365295e-11
    #goal.target_pose.pose.position.y = -4.768676564737007e-10

    #goal.target_pose.pose.orientation.z = 0.05482415109872818
    #goal.target_pose.pose.orientation.w = 0.9984959959983826
    
    
   






# #! /usr/bin/env python

# import rospy
# import actionlib
# import math
# from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# def movebase_client():

#     client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
#     client.wait_for_server()

#     goal = MoveBaseGoal()
#     goal.target_pose.header.frame_id = "map"
#     goal.target_pose.header.stamp = rospy.Time.now()
#     goal.target_pose.pose.position.x = float(input("Enter x coordinate: "))
#     goal.target_pose.pose.position.y = float(input("Enter y coordinate: "))

#     goal.target_pose.pose.orientation.z = math.pi/2
#     goal.target_pose.pose.orientation.w = 1

#     client.send_goal(goal)
#     wait = client.wait_for_result()
#     if not wait:
#         rospy.logerr("not available")
#         rospy.signal_shutdown("not available")
#     else:
#         return client.get_result()


# try:
#     rospy.init_node('movebase_client')
#     result = movebase_client()
#     if result:
#         rospy.loginfo("Goal execution done!")
# except rospy.ROSInterruptException:
#     rospy.loginfo("Navigation done")
    
    
    
#    #x: -2.5204879420925863e-05
#       #y: -2.3117668206396047e-06

      
      
#       #x: -2.5204879420925863e-05
#       #y: -2.3117668206396047e-06
      
#     #goal.target_pose.pose.position.x = 5.0163290188365295e-11
#     #goal.target_pose.pose.position.y = -4.768676564737007e-10

#     #goal.target_pose.pose.orientation.z = 0.05482415109872818
#     #goal.target_pose.pose.orientation.w = 0.9984959959983826
    
    
   



