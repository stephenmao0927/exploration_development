#!/usr/bin/env python

import rospy
import numpy
import tf2_ros
from move_base_msgs.msg import MoveBaseActionGoal
from geometry_msgs.msg import PoseStamped as PS
from tf2_geometry_msgs import PoseStamped as TPS

def receive_and_pack():
    receive_goal = rospy.wait_for_message(server_ns+'/move_base_simple/goal', PS)
    return receive_goal

def send(goal):
    source_pose = TPS()
    source_pose = goal
    print(source_pose)
    transformed_pose = tf_buffer.transform(source_pose, target_ns + "/map")
    nav_goal = MoveBaseActionGoal()
    nav_goal.goal.target_pose = transformed_pose
    print(nav_goal.goal.target_pose)
    pub.publish(nav_goal)

if __name__ == '__main__':
    rospy.init_node('transmit_nav_goal', anonymous=False)  
    server_ns = rospy.get_param('server_namespace','')
    target_ns = rospy.get_param('target_namespace','robot_1')      
    pub = rospy.Publisher(target_ns+'/move_base/goal', MoveBaseActionGoal, queue_size=10)
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)
    try:
        while not rospy.is_shutdown():
            goal = receive_and_pack()
            send(goal)
    except rospy.ROSInterruptException:
        pass

