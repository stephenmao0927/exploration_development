#!/usr/bin/env python3

import rospy
import tf
import numpy as np
from numpy import array
import actionlib
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import PoseStamped
from numpy import floor
from numpy.linalg import norm
from numpy import inf
from geometry_msgs.msg import Quaternion

# ________________________________________________________________________________

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

class robot:
    goal = MoveBaseGoal()
    start = PoseStamped()
    end = PoseStamped()

    def __init__(self, name):
        self.assigned_point = []
        self.name = name
        print('robot name: ', self.name)
        # self.global_frame = rospy.get_param('~global_frame', '/map')
        # self.global_frame = rospy.get_param('/assigner/global_frame', '')
        # print('global frame: ', self.global_frame)
        # self.robot_frame = rospy.get_param('/assigner/robot_frame', '')
        # print('robot frame: ', self.robot_frame)
        
        self.global_frame = self.name+'/map'
        self.robot_frame = self.name+'/base_footprint'

        self.plan_service = rospy.get_param(
            # '~plan_service', '/move_base_node/NavfnROS/make_plan')
            '~plan_service', '/move_base_node/NavfnROS/make_plan')

        self.listener = tf.TransformListener()
        self.listener.waitForTransform(
            # self.global_frame, self.name+'/'+self.robot_frame, rospy.Time(0), rospy.Duration(10.0))
            self.global_frame, self.robot_frame, rospy.Time(0), rospy.Duration(10.0))
        cond = 0
        while cond == 0:
            try:
                # print(self.global_frame, self.robot_frame)
                rospy.loginfo('Waiting for the robot transform')
                print(self.global_frame, self.robot_frame)
                (trans, rot) = self.listener.lookupTransform(
                    # self.global_frame, self.name+'/'+self.robot_frame, rospy.Time(0))
                    self.global_frame, self.robot_frame, rospy.Time(0))
                cond = 1
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                cond = 0
        
        self.position = array([trans[0], trans[1]])
        self.assigned_point = self.position
        self.client = actionlib.SimpleActionClient(
            '/'+self.name+'/move_base', MoveBaseAction)
        self.client.wait_for_server()

        rospy.wait_for_service('/'+self.name+self.plan_service)
        self.make_plan = rospy.ServiceProxy(
            self.name+self.plan_service, GetPlan)

        # rospy.logwarn('hey I am here')

        
        # robot.start.header.frame_id = self.name[1:]+'/map'
        # robot.end.header.frame_id = self.name[1:]+'/map'
        robot.start.header.frame_id = self.name+'/map'
        robot.end.header.frame_id = self.name+'/map'

    def getPosition(self):
        cond = 0
        while cond == 0:
            try:
                # (trans, rot) = self.listener.lookupTransform(
                #     self.global_frame, self.name+'/'+self.robot_frame, rospy.Time(0))
                (trans, rot) = self.listener.lookupTransform(
                     self.global_frame, self.robot_frame, rospy.Time(0))
                cond = 1
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                cond == 0
        self.position = array([trans[0], trans[1]])
        return self.position

    def sendGoal(self, point):
        robot_position = self.getPosition()
        # change frame id to corresponding robot
        # robot.goal.target_pose.header.frame_id = self.name[1:]+'/map'
        robot.goal.target_pose.header.frame_id = self.global_frame
        robot.goal.target_pose.header.stamp = rospy.Time.now()
        robot.goal.target_pose.pose.position.x = point[0]
        robot.goal.target_pose.pose.position.y = point[1]
        dx = point[0] - robot_position[0]
        dy = point[1] - robot_position[1]
        yaw_goal = math.atan2(dy, dx)
        robot.goal.target_pose.pose.orientation = quaternion_proceed(yaw_goal, 0, 0)
        self.client.send_goal(robot.goal)
        # print(robot.goal)
        self.assigned_point = array(point)

    def cancelGoal(self):
        self.client.cancel_goal()
        self.assigned_point = self.getPosition()

    def getState(self):
        return self.client.get_state()

    def makePlan(self, start, end):
        robot.start.pose.position.x = start[0]
        robot.start.pose.position.y = start[1]
        robot.end.pose.position.x = end[0]
        robot.end.pose.position.y = end[1]
        start = self.listener.transformPose(self.name+'/map', robot.start)
        end = self.listener.transformPose(self.name+'/map', robot.end)
        # start = self.listener.transformPose('/map', robot.start)
        # end = self.listener.transformPose('/map', robot.end)
        plan = self.make_plan(start=start, goal=end, tolerance=0.0)
        return plan.plan.poses
# ________________________________________________________________________________


def index_of_point(mapData, Xp):
    resolution = mapData.info.resolution
    Xstartx = mapData.info.origin.position.x
    Xstarty = mapData.info.origin.position.y
    width = mapData.info.width
    Data = mapData.data
    index = int(	(floor((Xp[1]-Xstarty)/resolution) *
                  width)+(floor((Xp[0]-Xstartx)/resolution)))
    return index


def point_of_index(mapData, i):
    y = mapData.info.origin.position.y + \
        (i/mapData.info.width)*mapData.info.resolution
    x = mapData.info.origin.position.x + \
        (i-(i/mapData.info.width)*(mapData.info.width))*mapData.info.resolution
    return array([x, y])
# ________________________________________________________________________________


def informationGain(mapData, point, r):
    infoGain = 0
    index = index_of_point(mapData, point)
    r_region = int(r/mapData.info.resolution)
    init_index = index-r_region*(mapData.info.width+1)
    for n in range(0, 2*r_region+1):
        start = n*mapData.info.width+init_index
        end = start+2*r_region
        limit = ((start/mapData.info.width)+2)*mapData.info.width
        for i in range(start, end+1):
            if (i >= 0 and i < limit and i < len(mapData.data)):
                if(mapData.data[i] == -1 and norm(array(point)-point_of_index(mapData, i)) <= r):
                    infoGain += 1
    return infoGain*(mapData.info.resolution**2)
# ________________________________________________________________________________


def discount(mapData, assigned_pt, centroids, infoGain, r):
    index = index_of_point(mapData, assigned_pt)
    r_region = int(r/mapData.info.resolution)
    init_index = index-r_region*(mapData.info.width+1)
    for n in range(0, 2*r_region+1):
        start = n*mapData.info.width+init_index
        end = start+2*r_region
        limit = ((start/mapData.info.width)+2)*mapData.info.width
        for i in range(start, end+1):
            if (i >= 0 and i < limit and i < len(mapData.data)):
                for j in range(0, len(centroids)):
                    current_pt = centroids[j]
                    if(mapData.data[i] == -1 and norm(point_of_index(mapData, i)-current_pt) <= r and norm(point_of_index(mapData, i)-assigned_pt) <= r):
                        # this should be modified, subtract the area of a cell, not 1
                        infoGain[j] -= 1
    return infoGain
# ________________________________________________________________________________


def pathCost(path):
    if (len(path) > 0):
        i = len(path)/2
        p1 = array([path[i-1].pose.position.x, path[i-1].pose.position.y])
        p2 = array([path[i].pose.position.x, path[i].pose.position.y])
        return norm(p1-p2)*(len(path)-1)
    else:
        return inf
# ________________________________________________________________________________


def unvalid(mapData, pt):
    index = index_of_point(mapData, pt)
    r_region = 5
    init_index = index-r_region*(mapData.info.width+1)
    for n in range(0, 2*r_region+1):
        start = n*mapData.info.width+init_index
        end = start+2*r_region
        limit = ((start/mapData.info.width)+2)*mapData.info.width
        for i in range(start, end+1):
            if (i >= 0 and i < limit and i < len(mapData.data)):
                if(mapData.data[i] == 1):
                    return True
    return False
# ________________________________________________________________________________


def Nearest(V, x):
    n = inf
    i = 0
    for i in range(0, V.shape[0]):
        n1 = norm(V[i, :]-x)
        if (n1 < n):
            n = n1
            result = i
    return result

# ________________________________________________________________________________


def Nearest2(V, x):
    n = inf
    result = 0
    for i in range(0, len(V)):
        n1 = norm(V[i]-x)

        if (n1 < n):
            n = n1
    return i
# ________________________________________________________________________________


def gridValue(mapData, Xp):
    resolution = mapData.info.resolution
    Xstartx = mapData.info.origin.position.x
    Xstarty = mapData.info.origin.position.y

    width = mapData.info.width
    Data = mapData.data
    # returns grid value at "Xp" location
    # map data:  100 occupied      -1 unknown       0 free
    index = (floor((Xp[1]-Xstarty)/resolution)*width) + \
        (floor((Xp[0]-Xstartx)/resolution))

    if int(index) < len(Data):
        return Data[int(index)]
    else:
        return 100

def GridCoordinate(mapData, p):
    ''' world coordinate to grid coordinate '''
    resolution = mapData.info.resolution
    startx = mapData.info.origin.position.x
    starty = mapData.info.origin.position.y
    gridplace = (int(np.floor((p[0]-startx)/resolution)),
            int(np.floor((p[1]-starty)/resolution)))
    return gridplace

def WorldCoordinate(mapData, list):
    output_list = []
    ''' grid coordinate to world coordinate '''
    resolution = mapData.info.resolution
    x = mapData.info.origin.position.x
    y = mapData.info.origin.position.y

    for i in range(len(list)):
        output_list.append((resolution*list[i][0]+ x, resolution*list[i][1]+ y))
    return output_list

def BresenhamLine(mapData, x1, y1, x2, y2):
    line = []
    processed_line = []
    dx = abs(x2 - x1)
    dy = abs(y2 - y1)
    
    s1 = 1 if ((x2 - x1) > 0) else -1
    s2 = 1 if ((y2 - y1) > 0) else -1

    boolInterChange = False
    if dy > dx:
        inter = dy
        dy = dx
        dx = inter
        boolInterChange = True

    line.append(np.array([x1,y1]))
    e = 2 * dy - dx
    x = x1
    y = y1
    for i in range(0, int(dx + 1)):
        if e >= 0:

            if boolInterChange:
                x += s1
            else:
                y += s2
            e -= 2 * dx

        if boolInterChange:
            y += s2
        else:
            x += s1
        e += 2 * dy
        line.append(np.array([x,y]))
    processed_line = WorldCoordinate(mapData, line)
    return processed_line, line

def GridValueList(mapData, list):
    ''' judge whether a set of pixels on 2d occupency map are all free spaces '''
    width = mapData.info.width
    # height = mapData.info.height
    Data = mapData.data

    for i in range(len(list)):
        index = list[i][1]*width + list[i][0]
        if Data[int(index)] == 0:
            continue
        else:
            return False
    return True

def GridValuePoint(mapData, p):
    ''' judge whether a single pixel on 2d occupency map is a free space '''
    width = mapData.info.width
    height = mapData.info.height
    Data = mapData.data
    index = p[1]*width + p[0]
    if Data[int(index)] == 0:
        return True
    else:
        return False