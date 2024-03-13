#!/usr/bin/env python3
#coding: utf-8 

import rospy
import math
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Quaternion

global now, goal, change, yaw

def quaternion_proceed(yaw, pitch, roll):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
 
    q = Quaternion()
    q.w = cy * cp * cr + sy * sp * sr
    q.x = cy * cp * sr - sy * sp * cr
    q.y = sy * cp * sr + cy * sp * cr
    q.z = sy * cp * cr - cy * sp * sr
    return q

def subscribe_goal():
    sub = rospy.Subscriber('centroids', Marker, caculate_position)

def caculate_position(msg):
    global goal, change, yaw
    goal = msg.points[0]
    yaw = math.atan2((goal.y - now.y),(goal.x - now.x))
    rospy.loginfo(yaw)
    change = np.array([0.2* math.cos(yaw), 0.2 * math.sin(yaw)])

def publish_odometry():
    pub = rospy.Publisher('/robot_1/odom', Odometry, queue_size=1)
    rate = rospy.Rate(5)  # 发布频率为5Hz
    
    while not rospy.is_shutdown():
        # 创建里程计消息
        odom = Odometry()
        
        # 设置里程计的header信息
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = 'map'
        
        # 设置里程计的位姿信息
        odom.pose.pose = Pose()
        odom.pose.pose.position.x = now.x + change[0]
        odom.pose.pose.position.y = now.y + change[1]
        odom.pose.pose.position.z = 0.0

        # 设置里程计的方向信息（使用四元数表示）
        quaternion = Quaternion()
        quaternion = quaternion_proceed(yaw, 0, 0)
        
        odom.pose.pose.orientation = quaternion
        
        # 发布里程计消息
        pub.publish(odom)
        now.x = odom.pose.pose.position.x
        now.y = odom.pose.pose.position.y
        
        rate.sleep()

def main():
    global now, change, goal, yaw
    now = Point()
    now.x = 10.0
    now.y = 37.5
    now.z = 0.0
    change = np.array([0, 0])
    yaw = 0
    rospy.init_node('odometry_publisher')
    while not rospy.is_shutdown():
        subscribe_goal()
        publish_odometry()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
