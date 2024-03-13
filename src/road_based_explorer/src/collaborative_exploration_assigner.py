#!/usr/bin/env python
#coding: utf-8 

import rospy
import numpy as np
import math
# import matplotlib.pyplot as plt
import tf2_ros
from scipy.spatial import Voronoi
from  nav_msgs.msg import Odometry
from tf2_geometry_msgs import PoseStamped as TPS
from geometry_msgs.msg import PolygonStamped, PointStamped, Point
from visualization_msgs.msg import Marker

def publish_quadrilateral(decided_points):
    quadrilateral_msg = PolygonStamped()
    quadrilateral_msg.header.stamp = rospy.Time.now()
    quadrilateral_msg.header.frame_id = 'robot_1/map'
    quadrilateral_msg.polygon.points = decided_points
    return quadrilateral_msg

def adjusted_arctan2(m,n):
    angle = np.arctan2(m,n)
    if angle <= 0:
        angle = 2 * math.pi + angle
        return angle
    else:
        return angle

def get_position():

    pose1 = rospy.wait_for_message("robot_1/Odometry", Odometry)
    pose2 = rospy.wait_for_message("robot_2/Odometry", Odometry)
    pose3 = rospy.wait_for_message("robot_3/Odometry", Odometry)

    pr1 = TPS()
    pr2 = TPS()
    pr2_ = TPS()
    pr3 = TPS()
    pr3_ = TPS()

    pr1.pose = pose1.pose.pose
    pr2_.pose = pose2.pose.pose
    pr3_.pose = pose3.pose.pose

    pr1.header = pose1.header
    pr2_.header = pose2.header
    pr3_.header = pose3.header
    pr2_.header.stamp = rospy.Time.now()
    pr2 = tf_buffer.transform(pr2_, "robot_1/map")

    pr3_.header.stamp = rospy.Time.now()
    pr3 = tf_buffer.transform(pr3_, "robot_1/map")

    return pr1, pr2, pr3

 # 计算直线方程
def line_equation(x1, y1, dx, dy):
    m = dy / dx
    A = m
    B = y1 - m * x1
    return A, B

def check_x(x, y, maxx, minx, vector):
    if (x <= maxx and x >= minx):
        vector.append((x, y))
    return vector

def check_y(x, y, maxy, miny, vector):
    if (y <= maxy and y >= miny):
        vector.append((x, y))
    return vector

def intersection_point(x1, y1, dx, dy, polygon):

    minx = polygon[0][0]
    maxx = polygon[1][0]
    maxy = polygon[0][1]
    miny = polygon[1][1]
    # 射线的直线方程
    A, B = line_equation(x1, y1, dx, dy)
    if abs(A)>100 and dy > 0:
        return (x1, maxy)
    elif abs(A)>100 and dy < 0:
        return (x1, miny)
    elif abs(A) < 0.01 and dx > 0:
        return (maxx, y1)
    elif abs(A) < 0.01 and dx < 0:
        return (minx, y1)
    else:
        intersection_points = []

        intersection_y = A * minx + B
        intersection_points = check_y(minx, intersection_y, maxy, miny, intersection_points)

        intersection_y = A * maxx + B
        intersection_points = check_y(maxx, intersection_y, maxy, miny, intersection_points)

        intersection_x = (maxy - B)/A
        intersection_points = check_x(intersection_x, maxy, maxx, minx, intersection_points)

        intersection_x = (miny - B)/A
        intersection_points = check_x(intersection_x, miny, maxx, minx, intersection_points)

        # 返回距离射线起点最近的交点
        if intersection_points:
            for i in range(len(intersection_points)):
                if (intersection_points[i][0] - x1) * dx > 0 and (intersection_points[i][1] - y1) * dy > 0:
                    # print('good')
                    return intersection_points[i]
        else:
            return None

def get_voronoi(pr1,pr2,pr3,polygon):
    calculated_points = []

    array_p1 = [pr1.pose.position.x, pr1.pose.position.y]
    array_p2 = [pr2.pose.position.x, pr2.pose.position.y]
    array_p3 = [pr3.pose.position.x, pr3.pose.position.y]

    points = np.array([array_p1, array_p2, array_p3])

    vor = Voronoi(points)

    center = vor.vertices[0] #包含Voronoi图的所有顶点的坐标。每个顶点是Voronoi区域的角点。
    vor.regions  #包含Voronoi图的各个区域
    vor.ridge_vertices #包含Voronoi图的脊线，即两个相邻Voronoi区域之间的边界线。
    vor.ridge_points #包含形成Voronoi图脊线的点对

    # print(center[0])
    # print(midp12[0])
    # k12 = (midp12[1] - center[1])/(midp12[0] - center[0])
    # k23 = (midp23[1] - center[1])/(midp23[0] - center[0])
    # k31 = (midp31[1] - center[1])/(midp31[0] - center[0])

    if len(center) > 0:
        midp12 = [0.5 * (array_p1[0] + array_p2[0]), 0.5 * (array_p1[1] + array_p2[1])]
        midp23 = [0.5 * (array_p3[0] + array_p2[0]), 0.5 * (array_p3[1] + array_p2[1])]
        midp31 = [0.5 * (array_p1[0] + array_p3[0]), 0.5 * (array_p1[1] + array_p3[1])]

        # 同一点发出的三条射线
        x1, y1 = center[0],center[1]

        # 三条射线的方向
        directions = [(midp12[0] - x1, midp12[1] - y1), (midp23[0] - x1, midp23[1] - y1), (midp31[0] - x1, midp31[1] - y1)]
        # directions = [(x1 - midp12[0], y1 - midp12[1]), (x1 - midp23[0], y1 - midp23[1]), (x1 - midp31[0], y1 - midp31[1] - y1)]
        # directions = [k12,k23,k31]

        # 计算三个多边形的顶点
        for i, (dx, dy) in enumerate(directions):
            intersection = intersection_point(x1, y1, dx, dy, polygon)    
            # print(i, intersection)
            calculated_points.append(intersection)

        # print(len(calculated_points))
            
        # angle_r = [adjusted_arctan2(polygon[0][1] - y1, polygon[0][0] - x1), 
        #            adjusted_arctan2(polygon[0][1] - y1, polygon[1][0] - x1),
        #            adjusted_arctan2(polygon[1][1] - y1, polygon[1][0] - x1),
        #            adjusted_arctan2(polygon[1][1] - y1, polygon[0][0] - x1)]

        # angle_12 = adjusted_arctan2(directions[0][1], directions[0][0])
        # angle_23 = adjusted_arctan2(directions[1][1], directions[1][0])
        # angle_31 = adjusted_arctan2(directions[2][1], directions[2][0])
        
        # sorted_angles = sorted([(angle_12, 'l12'), (angle_23, 'l23'), (angle_31, 'l31')])
        
        area_r1 = []
        area_r2 = []
        area_r3 = []
                
        corner = [(polygon[0][1], polygon[0][0]), 
                (polygon[0][1], polygon[1][0]),
                (polygon[1][1], polygon[1][0]),
                (polygon[1][1], polygon[0][0])]

        area_r1 = [(x1,y1),calculated_points[0],calculated_points[1]]
        area_r2 = [(x1,y1),calculated_points[1],calculated_points[2]]
        area_r3 = [(x1,y1),calculated_points[2],calculated_points[0]]

        for i in range(len(corner)):
            dis1  = math.sqrt(sum((x - y) ** 2 for x, y in zip(corner[i], array_p1)))
            dis2  = math.sqrt(sum((x - y) ** 2 for x, y in zip(corner[i], array_p2)))
            dis3  = math.sqrt(sum((x - y) ** 2 for x, y in zip(corner[i], array_p3)))
            # print(dis1, dis2, dis3)
            dis = sorted([dis1, dis2, dis3])
            if dis[0] == dis1:
                area_r1.append(corner[i])
            elif dis[0] == dis2:
                area_r2.append(corner[i])
            elif dis[0] == dis3:
                area_r3.append(corner[i])   
        
        return area_r1, area_r2, area_r3, center, calculated_points[0], calculated_points[1], calculated_points[2]
    
def generate_exploration_polygon(polygon):
    area_points = []
    x1, y1 = polygon[0][0], polygon[0][1]
    x2, y2 = polygon[1][0], polygon[1][1]

    # # Ensure x1, y1 is the top left corner
    # if x1 > x2:
    #     x1, x2 = x2, x1
    # if y1 > y2:
    #     y1, y2 = y2, y1
    
    # # Create the list of points in clockwise order
    # x1 = x1
    # x2 = x2
    # y1 = y1
    # y2 = y2

    polygon_points = [(x1, y1), (x2, y1), (x2, y2), (x1, y2)]

    # Iterate through the points and append them to the area_points list
    for point in polygon_points:
        area_point_msg = PointStamped()
        area_point_msg.point.x = point[0]
        area_point_msg.point.y = point[1]
        area_point_msg.point.z = 0
        area_points.append(area_point_msg.point)

    return area_points

def get_line_msg(point_1,point_2):

    # 创建一个Marker消息
    line_marker = Marker()
    line_marker.header.frame_id = "robot_1/map"  # 设置坐标系
    line_marker.type = Marker.LINE_STRIP  # 设置为直线
    line_marker.action = Marker.ADD
    line_marker.scale.x = 0.1  # 设置线的宽度

    # 添加直线上的点
    point1 = Point()
    point1.x = point_1[0]
    point1.y = point_1[1]
    point1.z = 0.0

    point2 = Point()
    point2.x = point_2[0]
    point2.y = point_2[1]
    point2.z = 1.0

    line_marker.points.append(point1)
    line_marker.points.append(point2)

    # 设置线的颜色
    line_marker.color.a = 1.0  # 不透明
    line_marker.color.r = 1.0  # 红色

    return line_marker
    

    
if __name__ == "__main__":
    try:
        rospy.init_node('collaborative_exploration_assigner', anonymous=True)
        rate = rospy.Rate(10)

        tf_buffer = tf2_ros.Buffer(rospy.Duration(300.0))
        listener = tf2_ros.TransformListener(tf_buffer)

        quadrilateral_pub = rospy.Publisher('quadrilateral', PolygonStamped, queue_size=1)
        marker_pub1 = rospy.Publisher('line_marker1', Marker, queue_size=10)
        marker_pub2 = rospy.Publisher('line_marker2', Marker, queue_size=10)
        marker_pub3 = rospy.Publisher('line_marker3', Marker, queue_size=10)

        tf_buffer = tf2_ros.Buffer(rospy.Duration(500.0))
        listener = tf2_ros.TransformListener(tf_buffer)
        polygon = np.array([[-20,5],[50,-55]])
        rect = generate_exploration_polygon(polygon)

        quadrilateral_msg = publish_quadrilateral(rect)

        while not rospy.is_shutdown():
            pr1,pr2,pr3 = get_position()
            area_r1,area_r2,area_r3, pc, p1, p2, p3= get_voronoi(pr1, pr2, pr3, polygon)
            print(pc, p1, p2, p3)
            line_msg1 = get_line_msg(pc, p1)
            line_msg2 = get_line_msg(pc, p2)
            line_msg3 = get_line_msg(pc, p3)
            # print(area_r1)
            # print(area_r2)
            # print(area_r3)
            quadrilateral_pub.publish(quadrilateral_msg)
            marker_pub1.publish(line_msg1)
            marker_pub2.publish(line_msg2)
            marker_pub3.publish(line_msg3)
    except rospy.ROSInterruptException:
        pass

