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
        odom3 = rospy.wait_for_message('/robot_3/odom', Odometry)
        yaw3 = yaw_proceed(odom3.pose.pose.orientation) 

        if odom3.pose.pose.position.x < 116 and odom3.pose.pose.position.y < 0.01 and not finish1:
            GoForward(3)
        elif odom3.pose.pose.position.x >= 116 and odom3.pose.pose.position.y < 0.01 and not finish2:
            finish1 = True
            Stop(3)
            while yaw3 > -1.55:
                odom3 = rospy.wait_for_message('/robot_3/odom', Odometry)
                yaw3 = yaw_proceed(odom3.pose.pose.orientation)
                Turn(3,"right")
            finish2 = True
            GoForward(3)
        elif odom3.pose.pose.position.x > 114 and odom3.pose.pose.position.y > -51 and not finish3:
            GoForward(3)
        elif odom3.pose.pose.position.x > 114 and odom3.pose.pose.position.y <= -51 and not finish4 :
            Stop(3)
            finish3 = True
            while yaw3 > -3.12:
                odom3 = rospy.wait_for_message('/robot_3/odom', Odometry)
                yaw3 = yaw_proceed(odom3.pose.pose.orientation)
                Turn(3,"right")
            GoForward(3)
            finish4 = True
        elif odom3.pose.pose.position.x > -6 and odom3.pose.pose.position.y < -48 and not finish5:
            GoForward(3)
        elif odom3.pose.pose.position.x <= -6 and odom3.pose.pose.position.y < -48 and not finish6:
            finish5 = True
            Stop(3)
            while yaw3 > 1.59 or ( yaw3 < 0 and yaw3 > -numpy.pi) :
                odom3 = rospy.wait_for_message('/robot_3/odom', Odometry)
                yaw3 = yaw_proceed(odom3.pose.pose.orientation)
                Turn(3,"right")
            GoForward(3)
            finish6 = True
        elif odom3.pose.pose.position.x < -2 and odom3.pose.pose.position.y < 2 and not finish7:
            GoForward(3)
        elif odom3.pose.pose.position.x <= -2 and odom3.pose.pose.position.y >= 2 and not finish8:
            finish7 = True
            Stop(3)
            while yaw3 > 0.02 :
                odom3 = rospy.wait_for_message('/robot_3/odom', Odometry)
                yaw3 = yaw_proceed(odom3.pose.pose.orientation)
                Turn(3,"right")
            GoForward(3)
            Stop(3)
            finish8 = True
        elif odom3.pose.pose.position.x < 14 and odom3.pose.pose.position.y < 4:
            GoForward(3)
        elif odom3.pose.pose.position.x >= 14 and odom3.pose.pose.position.y < 4:
            Stop(3)

if __name__ == '__main__':
    rospy.init_node('Move_Robot_3', anonymous=False)
    try:
        Move()
    except rospy.ROSInterruptException:
        pass
