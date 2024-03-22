#!/usr/bin/env python
# created and maintained by Stephen Mao (maok0002@e.ntu.edu.sg)

# --------Include modules---------------
import rospy
from copy import copy
from numpy import array
from rrt_exploration.msg import PointArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from road_based_explorer.srv import transfer_goal

goal_topic = rospy.get_param('server/goal_topic','')
name = rospy.get_param('server/robot_name','')

def get_goal(req):
    if req: 
        rospy.loginfo("Goal request confirmed")
        cen_send = rospy.wait_for_message(goal_topic, PointArray)
        now_odom = rospy.wait_for_message('/'+name+'/Odometry', Odometry)
        now_odom_array = array([now_odom.pose.pose.position.x,now_odom.pose.pose.position.y])

        odom_array = []
        odom_send = PointArray()
        
        odom_p = Point()

        odom_p.x = now_odom_array[0]
        odom_p.y = now_odom_array[1]

        for r in range(0, len(cen_send.points)):
            odom_array.append(copy(odom_p))

        # print(cen_send.points)
        odom_send.points = odom_array

        return cen_send, odom_send
            
# Node----------------------------------------------

def node():
    rospy.init_node('server', anonymous=True)
    goal_request = rospy.Service('transfer_goal', transfer_goal, get_goal)
    rospy.spin()
# -------------------------------------------------------------------------

if __name__ == '__main__':
    try:
        rospy.logwarn("goal server running")
        node()
    except rospy.ROSInterruptException:
        pass
