#!/usr/bin/env python3
#coding: utf-8 

import rospy
import tf2_ros
import numpy as np
import copy
import cv2
import math
from nav_msgs.msg import OccupancyGrid,Odometry
from tf2_geometry_msgs import PoseStamped
from geometry_msgs.msg import Quaternion,PointStamped,Point
from visualization_msgs.msg import Marker
from transforms3d.euler import quat2euler

class CostMap(object):
    """
    This is a class to get the Global Cost Map.
    """ 
    def __init__(self):
        global costmap_topic
        """
        Constructor for Global Cost map class.
        :param configs: configurations for costmap
        :type configs: OccupancyGrid
        """
        # self.occupancygrid_msg = None
        self.occupancygrid_msg = None
        self.map_data = None
        self.width = None
        self.height = None
        self.position = None
        
        costmap_topic = rospy.get_param('~costmap_topic', 'robot_1/map')
        rospy.Subscriber('/'+costmap_topic, OccupancyGrid, self.get_cost_map)
        print('received')

    def get_cost_map(self, msg):
        global costmap_frame
        """
        costmap subscriber's callback function.
        """
        self.occupancygrid_msg = msg
        self.map_data = msg.data
        self.width = msg.info.width
        self.height = msg.info.height
        self.position = msg.info.origin
        costmap_frame = msg.header.frame_id
        
    def get_map(self):
        """
        This function returns the size and 2D costmap.
        :rtype: [int, int], np.ndarray
        """
        while(not self.occupancygrid_msg):
            # print('not yet!')
            continue
        
        width = copy.deepcopy(self.width)
        height = copy.deepcopy(self.height)
        size = [width, height]
        costmap = copy.deepcopy(self.map_data)
        
        return size, costmap

def draw_map(map_size, map_data):
    global img,contours
    """
        This function returns the RGB image perceived by the camera.
        :rtype: np.ndarray or None
    """
    row, col = map_size[1], map_size[0]  
    costmap = np.array(map_data)
    costmap = costmap.reshape((row, col))
    img = np.zeros((row, col, 3))
    for i in range(row):
        for j in range(col):
        	# 判断无障碍点、未知点和障碍点 画不同颜色
            if costmap[i][j] == 0:
                img[i][j] = [255, 255, 255]
            # elif costmap[i][j] == -1:
            #     img[i][j] = [0, 0, 127]
            else:
                img[i][j] = [0, 0, 0]
    img_1 = img.astype(np.uint8)  # 转换为 uint8 类型
    img_2 = cv2.cvtColor(img_1, cv2.COLOR_BGR2GRAY)  # 转换为灰度图像
    contours, hierarchy = cv2.findContours(img_2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    return img, contours

def GridCoordinate(mapData, Xp):
    resolution = mapData.info.resolution
    Xstartx = mapData.info.origin.position.x
    Xstarty = mapData.info.origin.position.y
    # returns grid coordinate at "Xp" location
    # map data:  100 occupied      -1 unknown       0 free
    cellplace = (int(np.floor((Xp[0]-Xstartx)/resolution)),
             int(np.floor((Xp[1]-Xstarty)/resolution)))
    return cellplace

def WorldCoordinate(mapData, point):
    resolution = mapData.info.resolution
    Xstartx = mapData.info.origin.position.x
    Xstarty = mapData.info.origin.position.y
    worldplace = (resolution*point[0]+Xstartx, resolution*point[1]+Xstarty)
    return worldplace

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

def yaw_proceed(orientation):
    _, _, yaw = quat2euler(
            [orientation.w,
            orientation.x,
            orientation.y,
            orientation.z])
    return yaw

class Get_Odom(object):
    def __init__(self):
        self.frame_id = None
        self.origin = PoseStamped()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        rospy.sleep(0.1)
        rospy.Subscriber('/robot_1/odom', Odometry, self.get_odom)
    
    def check_transform(self,frame_id):
        global costmap_topic
        can_transform = self.tf_buffer.can_transform(costmap_topic, frame_id, rospy.Time(0), rospy.Duration(1.0))
        if can_transform:
            print("able to proceed coordinate transform")
        else:
            print("unable to proceed coordinate transform")
    
    def get_odom(self, msg):
        global costmap_topic
        self.origin.header = msg.header
        self.origin.pose = msg.pose.pose
        # print(self.origin)
        # self.check_transform(self.origin.header.frame_id)
        try:
            self.map_pose = self.tf_buffer.transform(self.origin, costmap_topic, timeout=rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn('Failed to transform Odometry message')
    
    
class Pick_Points(object):
    def __init__(self):
        global contours
        self.points = []
        self.step = None
        self.x = None
        self.y = None
        self.w = None
        self.h = None
        self.draw_min_external_rectangle(contours)
        # self.draw_max_internal_rectangle(contours)
        self.find_point()
    
    def draw_min_external_rectangle(self,contours):
        global img
        for cnt in contours:
            self.x, self.y, self.w, self.h = cv2.boundingRect(cnt)
        # 绘制矩形
        cv2.rectangle(img, (self.x, self.y+self.h), (self.x+self.w, self.y), (0, 255, 255))
        return self.x, self.y, self.w, self.h

    def draw_max_internal_rectangle(self,contours):
        contour = contours[0].reshape(len(contours[0]), 2)
        rect = []
 
        for i in range(len(contour)):
            x1, y1 = contour[i]
            for j in range(len(contour)):
                x2, y2 = contour[j]
                area = abs(y2 - y1) * abs(x2 - x1)
                rect.append(((x1, y1), (x2, y2), area))
    
        all_rect = sorted(rect, key=lambda x: x[2], reverse=True)
    
        if all_rect:
            best_rect_found = False
            index_rect = 0
            nb_rect = len(all_rect)
    
            while not best_rect_found and index_rect < nb_rect:
    
                rect = all_rect[index_rect]
                (x1, y1) = rect[0]
                (x2, y2) = rect[1]
    
                valid_rect = True
    
                x = min(x1, x2)
                while x < max(x1, x2) + 1 and valid_rect:
                    if any(img[y1, x]) == 0 or any(img[y2, x]) == 0:
                        valid_rect = False
                    x += 1
    
                y = min(y1, y2)
                while y < max(y1, y2) + 1 and valid_rect:
                    if any(img[y, x1]) == 0 or any(img[y, x2]) == 0:
                        valid_rect = False
                    y += 1
    
                if valid_rect:
                    best_rect_found = True
    
                index_rect += 1
    
            if best_rect_found:
                print((x1, y1), (x2, y2))
                cv2.rectangle(img, (x1, y1), (x2, y2), (255, 0, 0), 1)

            else:
                print("No rectangle fitting into the area")
 
        else:
            print("No rectangle found")

    def find_point(self):
        global start_location, end_location
        retval, pt1, pt2 = cv2.clipLine((self.x, self.y, self.w, self.h), start_location, end_location)
        # print(retval, pt1, pt2)
        # cv2.circle(img, pt1, 1, (0, 120, 55), 5)
        self.step = rospy.get_param('~step',70)
        # print(pt2[0], self.x + self.w - 1)
        if pt2[0] == self.x or pt2[0] == self.x + self.w - 1:
            shubian = True
        elif pt2[1] == self.y or pt2[1] == self.y + self.h-1:
            shubian = False
        else: 
            shubian = False
        
        point = [pt2[0],pt2[1]]
        self.points.append(point.copy())
        if shubian:
            while point[1] + self.step < self.y + self.h:
                point[1] = point[1] + self.step
                self.points.append(point.copy())
                cv2.circle(img, tuple(point), 1, (55, 0, 0), 5)
            point = [pt2[0],pt2[1]]
            while point[1] - self.step > self.y:
                point[1] = point[1] - self.step
                self.points.append(point.copy())
                cv2.circle(img, tuple(point), 1, (55, 0, 0), 5)
        else:
            while point[0] + self.step < self.x + self.w:
                point[0] = point[0] + self.step
                self.points.append(point.copy())
                cv2.circle(img, tuple(point), 1, (55, 0, 0), 5)
            point = [pt2[0],pt2[1]]
            while point[0] - self.step > self.x:
                point[0] = point[0] - self.step
                self.points.append(point.copy())
                cv2.circle(img, tuple(point), 1, (55, 0, 0), 5)
            
        rate.sleep()
        
class Publish(object):
    global w_points, costmap_frame
    def __init__(self):
        self.marker = Marker()
        # pub = rospy.Publisher('/detected_points', PointStamped, queue_size=10)
        pub2 = rospy.Publisher('/test_exploartion_points', Marker, queue_size=10)
        
        # print(w_points)
        for p in w_points:
            # msg = PointStamped()
            # msg.header.stamp = rospy.Time(0)
            # msg.header.frame_id = costmap_frame
            # msg.point.x = p[0]
            # msg.point.y = p[1]
            # msg.point.z = 0
            # pub.publish(msg)
            # rospy.sleep(0.2)
            
            point = Point()
            point.x = p[0]  # 设置点的x坐标
            point.y = p[1]  # 设置点的y坐标
            point.z = 0.0 
            self.marker.points.append(point)
        
        print(self.marker.points)
        self.marker.header.frame_id = costmap_frame  # 设置坐标系
        self.marker.header.stamp = rospy.Time.now()
        self.marker.id = 0
        self.marker.type = self.marker.POINTS  # 设置Marker类型为点
        self.marker.action = 0
        self.marker.scale.x = 1
        self.marker.scale.y = 1
        self.marker.color.r = 1.0
        self.marker.color.g = 0.0
        self.marker.color.b = 1.0
        self.marker.color.a = 1.0
        self.marker.lifetime = rospy.Duration(0)
        self.marker.points.append(point)
        print('hello')
        pub2.publish(self.marker)
        print('done')
        rate.sleep()

def main():
    global costmap_topic, img, contours, start_location, end_location, w_points
    while not rospy.is_shutdown():
        print('Start')
        costmap = CostMap()
        get_odom = Get_Odom()
        print('Load Map')
        map_size, map_data = costmap.get_map()	
        img, contours = draw_map(map_size, map_data)
        print('Finished!')
        
        start_location = GridCoordinate(costmap.occupancygrid_msg, (get_odom.map_pose.pose.position.x,get_odom.map_pose.pose.position.y))
        yaw = yaw_proceed(get_odom.map_pose.pose.orientation)
        endx = get_odom.map_pose.pose.position.x+max(costmap.height,costmap.width)*math.cos(yaw)
        endy = get_odom.map_pose.pose.position.y+max(costmap.height,costmap.width)*math.sin(yaw)
        end_location = GridCoordinate(costmap.occupancygrid_msg,(endx,endy))
        cv2.drawContours(img, contours, -1, (0,255,0), 2)
        cv2.arrowedLine(img, start_location, end_location, (0, 0, 255), 2, 9, 0, 0.3)  # 画箭头
        # # cv2.circle(img, start_location, 1, (0, 0, 255), 2)

        w_points = []
        pick = Pick_Points()
        for p in pick.points:
            w_points.append(WorldCoordinate(costmap.occupancygrid_msg,p.copy()))
            
        pub = Publish()

        # cv2.imshow('map',img)
        # # cv2.imwrite('/home/stephenmao/new_detector/6.jpg', img)
        # cv2.waitKey(0)

if __name__ == "__main__":
    try:
        rospy.init_node('local_detector', anonymous=True)
        rate = rospy.Rate(10)
        main()
    except rospy.ROSInterruptException:
        pass

