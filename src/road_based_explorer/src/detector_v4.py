#!/usr/bin/env python
#coding: utf-8 
# created and maintained by Stephen Mao (maok0002@e.ntu.edu.sg)

import rospy
import tf2_ros
import numpy as np
import copy
import math
from nav_msgs.msg import OccupancyGrid, Odometry
from tf2_geometry_msgs import PoseStamped
from geometry_msgs.msg import Quaternion, Point
from visualization_msgs.msg import Marker
from transforms3d.euler import quat2euler
from collections import deque
from rrt_exploration.msg import RoboArcArray,PointArray

costmap_topic = rospy.get_param('detector/costmap_topic', default='/')
odom_topic = rospy.get_param('detector/odometry_topic',default='/')
radius = rospy.get_param('detector/radius', default = 0)
number = rospy.get_param('detector/number', default = 0) 
robo_name = rospy.get_param('detector/robot_name', default = "/") 
global temp_radius

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
        self.msg = None
        
        # rospy.Subscriber(costmap_topic, OccupancyGrid, self.get_cost_map)
        self.get_map()
        self.get_cost_map()
        
    def get_map(self):
        while(not self.msg):
            # print('waiting for map')
            self.msg = rospy.wait_for_message(costmap_topic, OccupancyGrid)
            # print('received')
            continue

        width = copy.deepcopy(self.width)
        height = copy.deepcopy(self.height)
        size = [width, height]
        # print("Resolution  :::",self.msg.info.resolution)
        # costmap = copy.deepcopy(self.map_data)
        return size

    def get_cost_map(self):
        global costmap_frame
        """
        costmap subscriber's callback function.
        """
        self.occupancygrid_msg = self.msg
        self.map_data = self.msg.data
        self.width = self.msg.info.width
        self.height = self.msg.info.height
        self.position = self.msg.info.origin
        costmap_frame = self.msg.header.frame_id

class Get_Odom(object):
    global tf_buffer, tf_listener, map_pose
    def __init__(self):
        self.origin = PoseStamped()
        self.map_pose = None
        rospy.Subscriber(odom_topic, Odometry, self.get_odom)
    
    def check_transform(self,frame_id):
        global map_frame
        can_transform = tf_buffer.can_transform(map_frame, frame_id, rospy.Time(0), rospy.Duration(1.0))
        if can_transform:
            print("able to proceed coordinate transform")
        else:
            print("unable to proceed coordinate transform")
    
    def get_odom(self, msg):
        # global costmap_topic
        self.origin.header = msg.header
        # self.origin.header.frame_id = odom_frame
        self.origin.pose = msg.pose.pose
        self.map_pose = self.origin
        # # self.check_transform(self.origin.header.frame_id)
        # try:
        #     self.map_pose = tf_buffer.transform(self.origin, map_frame, timeout=rospy.Duration(1.0))
        # except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        #     rospy.logwarn('Failed to transform Odometry message')

    def get_map_pose(self):
        while(not self.map_pose):
            # print('waiting for odom')
            continue

        map_pose = self.map_pose
        return map_pose

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

def bfs(start, target_list):
    queue = deque([(start, 0)])  # 初始点和距离（距离初始化为0）
    visited = set()  # 已访问的点

    while queue:
        current, distance = queue.popleft()
        row, col = current

        if current in target_list:
            return current

        # 定义上、下、左、右四个方向的偏移量
        directions = [(0, 1), (0, -1), (1, 0), (-1, 0)]

        for dx, dy in directions:
            new_row, new_col = row + dx, col + dy

            # 检查新点是否在栅格范围内且未被访问过
            if (new_row, new_col) not in visited:
                queue.append(((new_row, new_col), distance + 1))
                visited.add((new_row, new_col))

    return None

class Pick_Points(object):
    def __init__(self, costmap_data, map_pose, map_size):
        # print(map_pose)
        self.costmap_data = costmap_data
        self.map_pose = map_pose
        self.map_size = map_size
        self.yaw = yaw_proceed(map_pose.pose.orientation)
        self.radius_suitbale = True
        self.circle_point_origin = []
        self.circle_point = []
        self.circle_point_value = []
        self.pick_point = [] 
        self.c_p_g = []
        self.all_world_list = []
        self.world_list_c_p = []
        self.first_pick = []
        self.delete = []
        self.final = []
        self.first_pick_world = []
        self.final_world = []
        self.radius = None
        self.c_p_g_b = None
        self.step = None

        for _ in range(8):
            self.circle_point_origin.append([])
        
        self.adjust_radius_from_filter()
        # rospy.loginfo("current radius for "+robo_name+" is "+str(self.radius))
        # while not self.radius_suitbale:
        self.robot_grid = self.GridCoordinate(self.costmap_data, (self.map_pose.pose.position.x, self.map_pose.pose.position.y))
        # 由机器人世界坐标，找到机器人在栅格地图中的离散坐标
        self.draw_circle(self.radius)
        self.deal_with_list()
        #用bresenham算法画圆并调整圆弧上点的索引值
        self.circle_point_value = self.GridValue(self.costmap_data, self.circle_point)
        #获取圆上所有点对应的cost
        # self.check_radius(self.costmap_data)
        # 检查采样半径是否符合要求
        self.find_crucial_point()
        #获取“关键点”(圆上车前进朝向的点和前进反方向朝向的点)栅格坐标
        self.all_world_list = self.WorldCoordinate(self.costmap_data, self.circle_point_value)
        #获取整个圆上点的世界坐标（测试用）
        self.world_list_c_p = self.WorldCoordinate(self.costmap_data, self.c_p_g)
        #获取关键点世界坐标
        self.step = len(self.all_world_list)/number
        self.sampling()
        # 采样
        # self.first_pick_world = self.WorldCoordinate(costmap_data, self.first_pick)
        #获取采样到的、所有cost值符合要求的点的世界坐标（测试用）
        self.final_world = self.WorldCoordinate_final(costmap_data, self.final)
        # 获取筛选完后的最终采样点

    def adjust_radius_from_filter(self):
        global temp_radius, radius
        need_adjust = rospy.get_param('~/'+robo_name+'/need_change_radius', '')
        if need_adjust == 'yes' and temp_radius > 15:
            temp_radius = temp_radius - 2
        elif need_adjust == 'no':
            temp_radius = radius
        elif temp_radius < 16:
            temp_radius = radius
        self.radius = temp_radius
        # rospy.loginfo("current radius for "+robo_name+" is "+str(temp_radius))


    
    def draw_circle(self, r_pixel):
        # 初始化,画第一个点，从水平最右边那个点开始画
        (x,y) = (r_pixel,0)
        """
        P_k=d1+d2
        d1 = 第1个下一步待选点离圆弧的距离（负数）
        d2 = 第2个下一步待选点离圆弧的距离（正数）
        但是为了提高效率通常使用递推来求P_{k+1}=P_k + 一个数
        """
        P_k = -2*r_pixel + 3

        # 迭代，求1/8圆弧
        while x>=y:
            # 两个待选点，具体选哪个取决于 P_k>=0 还是 <0
            if P_k>=0: 
                # 外侧候选点偏离圆弧更远
                P_k_next =  P_k - 4*x + 4*y + 10
                (x_next,y_next) = (x-1, y+1)
            else:
                # 内侧候选点偏离圆弧更远
                P_k_next =  P_k + 4*y + 6
                (x_next,y_next) = (x, y+1)
            # 对称法画其他地方
            self.draw_point(x, y, self.robot_grid[0], self.robot_grid[1], self.circle_point_origin[0])
            self.draw_point(-x, y, self.robot_grid[0], self.robot_grid[1], self.circle_point_origin[1]) 
            self.draw_point(x, -y, self.robot_grid[0], self.robot_grid[1], self.circle_point_origin[2]) 
            self.draw_point(-x, -y, self.robot_grid[0], self.robot_grid[1], self.circle_point_origin[3]) 

            self.draw_point(y, x, self.robot_grid[0], self.robot_grid[1], self.circle_point_origin[4]) 
            self.draw_point(y, -x, self.robot_grid[0], self.robot_grid[1], self.circle_point_origin[5]) 
            self.draw_point(-y, x, self.robot_grid[0], self.robot_grid[1], self.circle_point_origin[6]) 
            self.draw_point(-y, -x, self.robot_grid[0], self.robot_grid[1], self.circle_point_origin[7]) 
            # 更新坐标和P_k
            (x,y) = (int(x_next),int(y_next))
            P_k = P_k_next

    def draw_point(self, x, y, cx, cy, list):
        """ 
        绘制点(x,y)
        注意：需要把(x,y)变换到数组坐标系（图形学坐标系）
        因为数组(0,0)是左上，而原先坐标系(0,0)是中心点
        而且数组行数向下是增加的。
        """
        # 平移原点
        x += cx
        y += cy
        list.append((x,y))

    def deal_with_list(self):
        '''这个函数用来调整点的排序，使得总列表里相邻的点在圆的位置上也相邻'''
        list1 = list(reversed(self.circle_point_origin[4]))
        list1.pop(0)
        list2 = self.circle_point_origin[6]
        list2.pop(0)
        list3 = list(reversed(self.circle_point_origin[1]))
        list3.pop(0)
        list4 = self.circle_point_origin[3]
        list4.pop(0)
        list5 = list(reversed(self.circle_point_origin[7]))
        list5.pop(0)
        list6 = self.circle_point_origin[5]
        list6.pop(0)
        list7 = list(reversed(self.circle_point_origin[2]))
        list7.pop(0)
        list7.pop()
        self.circle_point = self.circle_point_origin[0] + list1 + list2 + list3 + list4 + list5 + list6 + list7
    
    def GridCoordinate(self, mapData, p):
        ''' world coordinate to grid coordinate '''
        resolution = mapData.info.resolution
        startx = mapData.info.origin.position.x
        starty = mapData.info.origin.position.y
        gridplace = (int(np.floor((p[0]-startx)/resolution)),
                int(np.floor((p[1]-starty)/resolution)))
        
        return gridplace

    def GridValue(self, mapData, list):
        ''' grid coordinate to grid value '''
        width = mapData.info.width
        height = mapData.info.height
        Data = mapData.data
        newlist = []
        for i in range(len(list)):
            if list[i][1] > height or list[i][0] > width:
                newlist.append((list[i][0], list[i][1], -999))
            else:
                index = list[i][1]*width + list[i][0]
                if index < len(Data):
                    newlist.append((list[i][0], list[i][1], Data[int(index)]))
                else:
                    newlist.append((list[i][0], list[i][1], -999))
        return newlist

    def WorldCoordinate(self, mapData, list):
        ''' grid coordinate to world coordinate '''
        resolution = mapData.info.resolution
        x = mapData.info.origin.position.x
        y = mapData.info.origin.position.y
        worldplace = []

        for i in range(len(list)):
            worldplace.append((resolution*list[i][0]+ x, resolution*list[i][1]+ y))
        
        return worldplace
    
    def WorldCoordinate_final(self, mapData, list_in):
        ''' grid coordinate to world coordinate '''
        resolution = mapData.info.resolution
        x = mapData.info.origin.position.x
        y = mapData.info.origin.position.y
        final_worldplace = []
        worldplace = []

        for point_list in list_in:
            for i in range(len(point_list)):
                worldplace.append((resolution*point_list[i][0]+ x, resolution*point_list[i][1]+ y))
            final_worldplace.append(worldplace)
            worldplace = []
        
        return final_worldplace
    
    def check_radius(self, mapData):
        global radius
        FREE = 0
        UNFREE = 0
        for i in self.circle_point_value:
            if i[2] == 0:
                FREE += 1
            else:
                UNFREE += 1
        ratio = FREE/UNFREE
        if ratio < 0.6 and ratio > 0.2:
            self.radius_suitbale = True
            radius = self.radius
            # rospy.set_param('~/robot_1/radius',radius * mapData.info.resolution)
        elif ratio >= 0.6:
            self.radius += 1
        elif ratio <= 0.2:
            self.radius -= 1
        rospy.loginfo('free space有'+str(FREE)+'个，其他栅格有'+str(UNFREE)+'个')

    def find_crucial_point(self):
        # offsets = [
        # (-1, -1), (-1, 0), (-1, 1),
        # (0, -1),           (0, 1),
        # (1, -1),  (1, 0),  (1, 1)
        # ]
        # c_p_w_f = (self.map_pose.pose.position.x + radius * self.costmap_data.info.resolution * math.cos(self.yaw) , self.map_pose.pose.position.y + radius * self.costmap_data.info.resolution * math.sin(self.yaw))
        c_p_w_b = (self.map_pose.pose.position.x - radius * self.costmap_data.info.resolution * math.cos(self.yaw) , self.map_pose.pose.position.y - radius * self.costmap_data.info.resolution * math.sin(self.yaw))
        # self.c_p_g_f = self.GridCoordinate(self.costmap_data, c_p_w_f)
        # if self.c_p_g_f not in self.circle_point:
        #     origin = list(self.c_p_g_f)    
        #     test = origin
        #     for dx, dy in offsets:
        #         test[0] = test[0] + dx
        #         test[1] = test[1] + dy
        #         if tuple(test) in self.circle_point:
        #             self.c_p_g_f = tuple(test)
        #             break
        #         else:
        #             self.c_p_g_f = origin 
        # self.c_p_g.append(self.c_p_g_f)

        self.c_p_g_b = self.GridCoordinate(self.costmap_data, c_p_w_b)
        self.c_p_g_b = bfs(self.c_p_g_b, self.circle_point)
        # if self.c_p_g_b not in self.circle_point:
        #     origin = list(self.c_p_g_b)
        #     test = origin
        #     for dx, dy in offsets:
        #         test[0] = test[0] + dx
        #         test[1] = test[1] + dy
        #         if tuple(test) in self.circle_point:
        #             self.c_p_g_b = tuple(test)
        #             break
        #         else:
        #             test = origin
            # c_p_w_b = (self.map_pose.pose.position.x - (radius+1) * self.costmap_data.info.resolution * math.cos(self.yaw) , self.map_pose.pose.position.y - radius * self.costmap_data.info.resolution * math.sin(self.yaw))
            # self.c_p_g_b = self.GridCoordinate(self.costmap_data, c_p_w_b)
        
        self.c_p_g.append(self.c_p_g_b)

    def sampling(self):
        i = self.circle_point.index(self.c_p_g_b)
        j = i
        while i < (len(self.circle_point_value)) and j < (len(self.circle_point_value)):
            if self.circle_point_value[int(j)][2] == 0:
                self.first_pick.append((self.circle_point[int(j)],int(j)))
            i += self.step
            j = round(i) 

        i = self.circle_point.index(self.c_p_g_b) - self.step
        j = round(i)
        while i >= 0:
            if self.circle_point_value[int(j)][2] == 0:
                self.first_pick.append((self.circle_point[int(j)],int(j)))
            i -= self.step
            j = round(i)

        # i = self.circle_point.index(self.c_p_g_b)
        # j = i
        # while self.circle_point_value[int(j)][2] == 0:
        #     self.delete.append((self.circle_point[int(j)],int(j)))
        #     i -= self.step
        #     j = round(i)
        #     if j >= 0:
        #             continue
        #     else:
        #         break

        # i = self.circle_point.index(self.c_p_g_b) + self.step
        # j = round(i)
        # if j < len(self.circle_point_value):
        #     while self.circle_point_value[int(j)][2] == 0:
        #         self.delete.append((self.circle_point[int(j)],int(j)))
        #         i += self.step
        #         j = round(i)
        #         if j < len(self.circle_point_value):
        #             continue
        #         else:
        #             break
    
        # second = list(set(self.first_pick) - set(self.delete))

        sorted_tuples = sorted(self.first_pick, key=lambda x: x[1])  # 按第二个元素（整数值）升序排序

        if len(sorted_tuples) > 0:
            current_list = [sorted_tuples[0][0]]  # 创建第一个列表，并将第一个坐标元组加入其中

            for i in range(1, len(sorted_tuples)):
                if sorted_tuples[i][1] - sorted_tuples[i - 1][1] <= round(len(self.circle_point_value)/number)+1:
                    current_list.append(sorted_tuples[i][0])
                else:
                    self.final.append(current_list)
                    current_list = [sorted_tuples[i][0]]

        # 添加最后一个列表
            if i < len(sorted_tuples):
                if len(self.circle_point_value)-sorted_tuples[i][1] <= round(len(self.circle_point_value)/number) and len(self.final) > 0:
                    self.final[0] = self.final[0] + current_list
                else:
                    self.final.append(current_list)
            else:
                self.final.append(current_list)
        


class Publish(object):
    def __init__(self, world_list):
        # self.marker1 = Marker()
        self.arrayarcs = RoboArcArray()
        self.world_list = world_list
        # pub = rospy.Publisher('/marker_detected_points', Marker, queue_size=10)
        pub2 = rospy.Publisher('/'+robo_name+'/detected_points', RoboArcArray, queue_size=10)

        # for p in self.world_list:
            
        #     point = Point()
        #     point.x = p[0]  # 设置点的x坐标
        #     point.y = p[1]  # 设置点的y坐标
        #     point.z = 0.0 
        #     self.marker1.points.append(point)
        
        # self.marker1.header.frame_id = costmap_frame  # 设置坐标系
        # self.marker1.header.stamp = rospy.Time.now()
        # self.marker1.id = 0
        # self.marker1.type = self.marker1.POINTS  # 设置Marker类型为点
        # self.marker1.action = 0
        # self.marker1.scale.x = 0.1
        # self.marker1.scale.y = 0.1
        # self.marker1.color.r = 1.0
        # self.marker1.color.g = 0.0
        # self.marker1.color.b = 1.0
        # self.marker1.color.a = 1.0
        # self.marker1.lifetime = rospy.Duration(0)

        for q in self.world_list:
            self.arc = PointArray()
            for r in q:
                point = Point()
                point.x = r[0]  # 设置点的x坐标
                point.y = r[1]  # 设置点的y坐标
                point.z = 0.0 
                self.arc.points.append(point)
            self.arrayarcs.arcs.append(self.arc)
        
        self.arrayarcs.name = robo_name

        # pub.publish(self.marker1)
        pub2.publish(self.arrayarcs)
        rate.sleep()

def main():
    global tf_buffer, tf_listener,temp_radius
    temp_radius = radius
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    map_pose = PoseStamped()
    while not rospy.is_shutdown(): 
        costmap = CostMap()
        map_size = costmap.get_map()
        get_odom = Get_Odom()
        map_pose = get_odom.get_map_pose()
        # 如果用simulated_city_two_robot测试的话，节点运行时间推进，robot_odom的发布频率变慢
        # 用一个函数来调节时序
        pick = Pick_Points(costmap.occupancygrid_msg, map_pose, map_size)
        # print(pick.final_world)
        pub = Publish(pick.final_world)
        
if __name__ == "__main__":
    try:
        rospy.init_node('detector', anonymous=True)
        rate = rospy.Rate(20)
        main()
    except rospy.ROSInterruptException:
        pass

