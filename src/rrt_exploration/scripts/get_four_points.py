#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PointStamped,Point
from std_msgs.msg import Header

if __name__ == '__main__':
    rospy.init_node('point_publisher')
    pub = rospy.Publisher('/clicked_point', PointStamped, queue_size=1)
    rate = rospy.Rate(1)  

    point1 = PointStamped()
    point1.header = Header()  
    point1.header.stamp = rospy.Time.now()
    point1.header.frame_id = "/robot_1/map"  
    point1.point.x = -5.0
    point1.point.y = 5.0
    point1.point.z = 0.0

    point2 = PointStamped()
    point2.header = Header()  
    point2.header.stamp = rospy.Time.now()
    point2.header.frame_id = "/robot_1/map"  
    point2.point.x = 30.0
    point2.point.y = 5.0
    point2.point.z = 0.0

    point3 = PointStamped()
    point3.header = Header()  
    point3.header.stamp = rospy.Time.now()
    point3.header.frame_id = "/robot_1/map"  
    point3.point.x = 30.0
    point3.point.y = -20.0
    point3.point.z = 0.0

    point4 = PointStamped()
    point4.header = Header()  
    point4.header.stamp = rospy.Time.now()
    point4.header.frame_id = "/robot_1/map"  
    point4.point.x = -5.0
    point4.point.y = -20.0
    point4.point.z = 0.0

    point5 = PointStamped()
    point5.header = Header()  
    point5.header.stamp = rospy.Time.now()
    point5.header.frame_id = "/robot_1/map"  
    point5.point.x = 0.0
    point5.point.y = 0.0
    point5.point.z = 0.0

    rospy.sleep(1)
    pub.publish(point1)
    rospy.sleep(1)
    pub.publish(point2)
    rospy.sleep(1)
    pub.publish(point3)
    rospy.sleep(1)
    pub.publish(point4)
    rospy.sleep(1)
    pub.publish(point5)

    rate.sleep()