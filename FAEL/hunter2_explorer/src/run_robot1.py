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
        odom1 = rospy.wait_for_message('/robot_1/odom', Odometry)
        yaw1 = yaw_proceed(odom1.pose.pose.orientation) 

        if odom1.pose.pose.position.x < 39 and odom1.pose.pose.position.y < 0.01 and not finish1:
            GoForward(1)
        elif odom1.pose.pose.position.x >= 39 and odom1.pose.pose.position.y < 0.01 and not finish2:
            finish1 = True
            Stop(1)
            while yaw1 > -1.55:
                odom1 = rospy.wait_for_message('/robot_1/odom', Odometry)
                yaw1 = yaw_proceed(odom1.pose.pose.orientation)
                Turn(1,"right")
            finish2 = True
            GoForward(1)
        elif odom1.pose.pose.position.x > 37 and odom1.pose.pose.position.y > -100 and not finish3:
            GoForward(1)
        elif odom1.pose.pose.position.x > 37 and odom1.pose.pose.position.y <= -100 and not finish4 :
            Stop(1)
            finish3 = True
            while yaw1 > -3.12:
                odom1 = rospy.wait_for_message('/robot_1/odom', Odometry)
                yaw1 = yaw_proceed(odom1.pose.pose.orientation)
                Turn(1,"right")
            GoForward(1)
            finish4 = True
        elif odom1.pose.pose.position.x > -7.5 and odom1.pose.pose.position.y < -95 and not finish5:
            GoForward(1)
        elif odom1.pose.pose.position.x <= -7.5 and odom1.pose.pose.position.y < -95 and not finish6:
            finish5 = True
            Stop(1)
            while yaw1 > 1.59 or ( yaw1 < 0 and yaw1 > -numpy.pi) :
                odom1 = rospy.wait_for_message('/robot_1/odom', Odometry)
                yaw1 = yaw_proceed(odom1.pose.pose.orientation)
                Turn(1,"right")
            GoForward(1)
            finish6 = True
        elif odom1.pose.pose.position.x < -4 and odom1.pose.pose.position.y < 1 and not finish7:
            GoForward(1)
        elif odom1.pose.pose.position.x < -4 and odom1.pose.pose.position.y >= 1 and not finish8:
            finish7 = True
            Stop(1)
            while yaw1 > 0.02 :
                odom1 = rospy.wait_for_message('/robot_1/odom', Odometry)
                yaw1 = yaw_proceed(odom1.pose.pose.orientation)
                Turn(1,"right")
            GoForward(1)
            finish8 = True
        elif odom1.pose.pose.position.x < 12 and odom1.pose.pose.position.y < 4:
            GoForward(1)
        elif odom1.pose.pose.position.x >= 12 and odom1.pose.pose.position.y < 4:
            Stop(1)

if __name__ == '__main__':
    rospy.init_node('Move_robot_1', anonymous=False)
    try:
        Move()
    except rospy.ROSInterruptException:
        pass

