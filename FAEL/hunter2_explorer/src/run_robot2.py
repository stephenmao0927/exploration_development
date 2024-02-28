#!/usr/bin/env python3

import rospy
import numpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from transforms3d.euler import quat2euler

def yaw_proceed(orientation):
    _, _, yaw = quat2euler(
            [orientation.w,
            orientation.x,
            orientation.y,
            orientation.z])
    return yaw  

class GoForward():
    def __init__(self, index):
        self.index = index
        self.cmd_vel = rospy.Publisher('robot_'+ str(self.index) +'/cmd_vel', Twist, queue_size=10)
        
        r = rospy.Rate(10)
        move_cmd = Twist()
        move_cmd.linear.x = 2.0
        move_cmd.angular.z = 0

        self.cmd_vel.publish(move_cmd)
        r.sleep()                       

class Stop():
    def __init__(self, index):
        self.index = index
        self.cmd_vel = rospy.Publisher('robot_'+ str(self.index) +'/cmd_vel', Twist, queue_size=10)
        
        r = rospy.Rate(10)
        move_cmd = Twist()
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 0

        self.cmd_vel.publish(move_cmd)
        r.sleep()
    
class Turn():
    def __init__(self, index, direction):
        self.index = index
        self.direction = direction
        self.cmd_vel = rospy.Publisher('robot_'+ str(self.index) +'/cmd_vel', Twist, queue_size=10)
        
        r = rospy.Rate(10)
        move_cmd = Twist()
        if direction == "left":
            move_cmd.linear.x = 0.0
            move_cmd.angular.z = 0.5
        
        if direction == "right":
            move_cmd.linear.x = 0.0
            move_cmd.angular.z = -0.5

        self.cmd_vel.publish(move_cmd)
        r.sleep()

def Move():
    finish1 = False
    finish2 = False
    finish3 = False
    finish4 = False
    finish5 = False
    finish6 = False
    finish7 = False
    finish8 = False
    while True:
        odom2 = rospy.wait_for_message('/robot_2/odom', Odometry)
        yaw2 = yaw_proceed(odom2.pose.pose.orientation) 

        if odom2.pose.pose.position.x < 42 and odom2.pose.pose.position.y < 0.01 and not finish1:
            GoForward(2)
        elif odom2.pose.pose.position.x >= 42 and odom2.pose.pose.position.y < 0.01 and not finish2:
            finish1 = True
            Stop(2)
            while yaw2 > -1.55:
                odom2 = rospy.wait_for_message('/robot_2/odom', Odometry)
                yaw2 = yaw_proceed(odom2.pose.pose.orientation)
                Turn(2,"right")
            finish2 = True
            GoForward(2)
        elif odom2.pose.pose.position.x > 39 and odom2.pose.pose.position.y > -51 and not finish3:
            GoForward(2)
        elif odom2.pose.pose.position.x > 39 and odom2.pose.pose.position.y <= -51 and not finish4 :
            Stop(2)
            finish3 = True
            while yaw2 > -3.12:
                odom2 = rospy.wait_for_message('/robot_2/odom', Odometry)
                yaw2 = yaw_proceed(odom2.pose.pose.orientation)
                Turn(2,"right")
            GoForward(2)
            finish4 = True
        elif odom2.pose.pose.position.x > -46 and odom2.pose.pose.position.y < -45 and not finish5:
            GoForward(2)
        elif odom2.pose.pose.position.x <= -46 and odom2.pose.pose.position.y < -45 and not finish6:
            finish5 = True
            Stop(2)
            while yaw2 > 1.59 or ( yaw2 < 0 and yaw2 > -numpy.pi) :
                odom2 = rospy.wait_for_message('/robot_2/odom', Odometry)
                yaw2 = yaw_proceed(odom2.pose.pose.orientation)
                Turn(2,"right")
            GoForward(2)
            finish6 = True
        elif odom2.pose.pose.position.x < -42 and odom2.pose.pose.position.y < 2 and not finish7:
            GoForward(2)
        elif odom2.pose.pose.position.x < -42 and odom2.pose.pose.position.y >= 2 and not finish8:
            finish7 = True
            Stop(2)
            while yaw2 > 0.02 :
                odom2 = rospy.wait_for_message('/robot_2/odom', Odometry)
                yaw2 = yaw_proceed(odom2.pose.pose.orientation)
                Turn(2,"right")
            GoForward(2)
            finish8 = True
        elif odom2.pose.pose.position.x < 16 and odom2.pose.pose.position.y < 4:
            GoForward(2)
        elif odom2.pose.pose.position.x >= 16 and odom2.pose.pose.position.y < 4:
            Stop(2)

if __name__ == '__main__':
    rospy.init_node('Move_Robot_2', anonymous=False)
    try:
        Move()
    except rospy.ROSInterruptException:
        pass

