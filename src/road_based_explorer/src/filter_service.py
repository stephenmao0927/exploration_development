#!/usr/bin/env python
# created and maintained by Stephen Mao (maok0002@e.ntu.edu.sg)

# --------Include modules---------------
from copy import copy
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PointStamped, PolygonStamped
import tf
from numpy import array, vstack, delete, linalg
from sklearn.cluster import MeanShift
from road_based_explorer.msg import RoboArcArray
from rrt_exploration.msg import PointArray
import ast
import matplotlib.pyplot as plt
from functions_service import GridCoordinate, BresenhamLine, GridValueList, GridValuePoint

# fetching all parameters
map_topic = rospy.get_param('filter/map_topic', '')
goals_topic = rospy.get_param('filter/goals_topic', '')
rateHz = rospy.get_param('filter/rate', 100)
robot_frame = rospy.get_param('filter/robot_frame', '')
little_gain_distance = rospy.get_param('filter/little_gain_distance', '')
namespace_init_count = rospy.get_param('namespace_init_count', 1)
n_robots = rospy.get_param('n_robots', 1)
robot_name = rospy.get_param('filter/robot_name', '')
exploration_points = rospy.get_param('filter/exploration_points', '')
print('exploration_points: ', exploration_points)

mapData = OccupancyGrid()
ready_for_goal = None
robot_state = None
radius_changing = False
frontiers = []
all_path = []
area_points = []
area_points_inside = []
odom_record = []
proceeded_frontiers = []

def WorldCoordinate(mapData, point):
    ''' grid coordinate to world coordinate '''
    resolution = mapData.info.resolution
    x = mapData.info.origin.position.x
    y = mapData.info.origin.position.y

    final_worldplace = array((resolution*point[0]+ x, resolution*point[1]+ y))
    
    return final_worldplace

class GetArea(object):
    global area_points, area_points_inside
    def __init__(self):
        self.distance = little_gain_distance
        rospy.init_node('filter', anonymous=True)
        rospy.loginfo("Please make sure that the points are published in clockwise order or in counterclockwise order!")
        self.points_received = False

        # self.polygons_sub = rospy.Subscriber('/clicked_point', PointStamped, self.point_callback)
        
        self.generate_exploration_polygon()

        self.get_inside_quadrilateral()

    def point_callback(self, point_msg):
        if point_msg.point not in area_points:
            area_points.append(point_msg.point)

        if len(area_points) == 4:
            self.points_received = True
            self.polygons_sub.unregister()
            self.get_inside_quadrilateral()

    def generate_exploration_polygon(self):

        # Separate the points
        polygon_points = ast.literal_eval(exploration_points)
        print("top left is %s, bottom right is %s", polygon_points[0], polygon_points[1])
        x1, y1 = polygon_points[0]
        x2, y2 = polygon_points[1]

        # Ensure x1, y1 is the top left corner
        if x1 > x2:
            x1, x2 = x2, x1
        if y1 > y2:
            y1, y2 = y2, y1
        
        # Create the list of points in clockwise order
        self.x1 = x1
        self.x2 = x2
        self.y1 = y1
        self.y2 = y2

        polygon_points = [(x1, y1), (x2, y1), (x2, y2), (x1, y2)]

        # Iterate through the points and append them to the area_points list
        for point in polygon_points:
            area_point_msg = PointStamped()
            area_point_msg.point.x = point[0]
            area_point_msg.point.y = point[1]
            area_point_msg.point.z = 0
            area_points.append(area_point_msg.point)

        # Set the points_received flag to True to start the exploration
        self.points_received = True

        rospy.loginfo("[%s]: exploration area is %s", robot_name, area_points)

    def get_inside_quadrilateral(self):

        polygon_points_inside = [(self.x1 + self.distance, self.y1 + self.distance), (self.x2 - self.distance, self.y1 + self.distance), 
        (self.x2 - self.distance, self.y2 - self.distance), (self.x1 + self.distance, self.y2 - self.distance)]

        for point in polygon_points_inside:
            inside_area_point_msg = PointStamped()
            inside_area_point_msg.point.x = point[0]
            inside_area_point_msg.point.y = point[1]
            inside_area_point_msg.point.z = 0
            area_points_inside.append(inside_area_point_msg.point)


def publish_quadrilateral(decided_points):
    quadrilateral_msg = PolygonStamped()
    quadrilateral_msg.header.stamp = rospy.Time.now()
    quadrilateral_msg.header.frame_id = name+'/map'
    quadrilateral_msg.polygon.points = decided_points
    return quadrilateral_msg

def callBack(data, args):
    global frontiers, name
    # transformedPoint = args[0].transformPoint(args[1], data)
    frontiers = []
    name = data.name
    for arc in data.arcs:
        arc_frontier = []
        for i in range(len(arc.points)):
            arc_frontier.append((arc.points[i].x,arc.points[i].y))
        frontiers.append(arc_frontier)

def mapCallBack(data):
    global mapData
    mapData = data
        
def is_point_inside_polygon(point, corner_points):
    x, y = point[0], point[1]
    j = 3
    odd_nodes = False

    poly_points = [(p.x, p.y) for p in corner_points]
    for i in range(4):
        xi, yi = poly_points[i]
        xj, yj = poly_points[j]

        if (yi < y and yj >= y) or (yj < y and yi >= y):
            if xi + (y - yi) / (yj - yi) * (xj - xi) < x:
                odd_nodes = not odd_nodes
        j = i

    return odd_nodes

def too_close_to_border(point, corner_points):
    x, y = point[0], point[1]
    odd_nodes = False

    poly_points = [(p.x, p.y) for p in corner_points]
    for i in range(4):
        xi, yi = poly_points[i]

        if (abs(x - xi) < 1) or (abs(y - yi) < 1):
            odd_nodes = True
            break

    return odd_nodes
            
# Node----------------------------------------------

def node():
    global centroids, frontiers, name, mapData, all_path, now_odom_array
    name = None
    rate = rospy.Rate(rateHz)

# ---------------------------------------------------------------------------------------------------------------
# wait if map is not received yet    
    rospy.Subscriber(map_topic, OccupancyGrid, mapCallBack)
    while (len(mapData.data) < 1):
        # rospy.loginfo('Waiting for the map')
        # rospy.sleep(0.1)
        pass

    global_frame = "/"+mapData.header.frame_id
    
    # rospy.logwarn(global_frame)
    # rospy.logwarn(robot_frame)
# ---------------------------------------------------------------------------------------------------------------
# carry out tf transform
    tfLisn = tf.TransformListener()
    tfLisn.waitForTransform(
        global_frame, '/'+robot_frame, rospy.Time(0), rospy.Duration(10.0))
    
    rospy.Subscriber(goals_topic, RoboArcArray, callback=callBack,
                    callback_args=[tfLisn, global_frame[1:]])
    
# ---------------------------------------------------------------------------------------------------------------
# wait if frontier is not received yet    
    while(not name):
            # print('waiting for name')
            # rospy.sleep(0.1)
            continue
        
    # rospy.Subscriber('/'+name+'/move_base/status', GoalStatusArray, goal_status_callback)
    pub = rospy.Publisher('frontiers', Marker, queue_size=10)
    pub3 = rospy.Publisher('updating_centroids', Marker, queue_size=10)
    see_path = rospy.Publisher('path_show', Marker, queue_size=10)
    see_bresenham = rospy.Publisher('bresenham_show', Marker, queue_size=10)
    filterpub = rospy.Publisher('filtered_points', PointArray, queue_size=10)
    quadrilateral_pub = rospy.Publisher('quadrilateral', PolygonStamped, queue_size=1)
    quadrilateral_pub_2 = rospy.Publisher('quadrilateral_inside', PolygonStamped, queue_size=1)
    quadrilateral_msg = publish_quadrilateral(area_points)
    quadrilateral_msg_2 = publish_quadrilateral(area_points_inside)

    rospy.loginfo("the map and global costmaps are received")

    # wait if no frontier is received yet
    while len(frontiers) < 1:
        pass

    points = Marker()
    centroid_marker = Marker()
    path_points = Marker()
    bresenham_points = Marker()

    path_points.header.frame_id = mapData.header.frame_id
    path_points.header.stamp = rospy.Time.now()
    path_points.ns = "path_point_list"
    path_points.id = 0
    path_points.type = Marker.POINTS
    path_points.action = Marker.ADD
    path_points.action = Marker.ADD
    path_points.pose.orientation.w = 1.0
    path_points.scale.x = 0.1
    path_points.scale.y = 0.1
    path_points.color.r = 0.0/255.0
    path_points.color.g = 0.0/255.0
    path_points.color.b = 255.0/255.00
    path_points.color.a = 1
    path_points.lifetime = rospy.Duration()
    path_p = Point()
    path_p.z = 0

    bresenham_points.header.frame_id = mapData.header.frame_id
    bresenham_points.header.stamp = rospy.Time.now()
    bresenham_points.ns = "bresenham_line_point_list"
    bresenham_points.id = 0
    bresenham_points.type = Marker.POINTS
    bresenham_points.action = Marker.ADD
    bresenham_points.action = Marker.ADD
    bresenham_points.pose.orientation.w = 1.0
    bresenham_points.scale.x = 0.1
    bresenham_points.scale.y = 0.1
    bresenham_points.color.r = 0.0/255.0
    bresenham_points.color.g = 0.0/255.0
    bresenham_points.color.b = 255.0/255.00
    bresenham_points.color.a = 1
    bresenham_points.lifetime = rospy.Duration()
    bresenham_p = Point()
    bresenham_p.z = 0

    centroid_marker.header.frame_id = mapData.header.frame_id
    centroid_marker.header.stamp = rospy.Time.now()
    centroid_marker.ns = "centroid_list"
    centroid_marker.id = 0
    centroid_marker.type = Marker.POINTS
    centroid_marker.action = Marker.ADD
    centroid_marker.action = Marker.ADD
    centroid_marker.pose.orientation.w = 1.0
    centroid_marker.scale.x = 0.2
    centroid_marker.scale.y = 0.2
    centroid_marker.color.r = 0.0/255.0
    centroid_marker.color.g = 255.0/255.0
    centroid_marker.color.b = 0.0/255.0

    centroid_marker.color.a = 1
    centroid_marker.lifetime = rospy.Duration()

    cen = Point()
    cen.z = 0

    points.header.frame_id = mapData.header.frame_id
    points.header.stamp = rospy.Time.now()
    points.ns = "markers2"
    points.id = 0
    points.type = Marker.POINTS
    points.action = Marker.ADD
    points.pose.orientation.w = 1.0
    points.scale.x = 0.2
    points.scale.y = 0.2
    points.color.r = 255.0/255.0
    points.color.g = 255.0/255.0
    points.color.b = 0.0/255.0
    points.color.a = 1
    points.lifetime = rospy.Duration()

    p = Point()
    p.z = 0

    c = Point()
    c.z = 0

    pp = []

    temppoint = PointStamped()
    temppoint.header.frame_id = mapData.header.frame_id
    temppoint.header.stamp = rospy.Time(0)
    temppoint.point.z = 0.0
    arraypoints = PointArray()
    tempPoint = Point()
    tempPoint.z = 0.0

# -------------------------------------------------------------------------
# ---------------------     Main   Loop     -------------------------------
# -------------------------------------------------------------------------
    while not rospy.is_shutdown():
# -------------------------------------------------------------------------
        # rospy.logwarn("filter is running")
        proceeded_frontiers = []
        centroids = []
        all_bresenham = []
        front = copy(frontiers)
        # rospy.logwarn("the number of arc is " + str(len(front)))

        odom = rospy.wait_for_message('/'+name+'/Odometry', Odometry)
        now_odom_array = array([odom.pose.pose.position.x,odom.pose.pose.position.y])

        if len(front) > 0:
            for arc in front:
                front.remove(arc)
                arc_cache = []
                for arc_i in range(len(arc)):
                    if is_point_inside_polygon(arc[arc_i], area_points_inside):
                        proceeded_frontiers.append(arc[arc_i])
                        arc_cache.append(arc[arc_i])
                    elif arc_i > 0 and is_point_inside_polygon(arc[arc_i-1], area_points_inside):
                        front.append(arc_cache)
                        arc_cache = []
                    else:
                        continue
                if arc_cache:
                    front.append(arc_cache)

            # rospy.logwarn(str(len(front)))
            # print(front)
            for arc in front:  
                sumx = 0
                sumy = 0
                for i in range(len(arc)):
                    sumx += arc[i][0]
                    sumy += arc[i][1]

                new_centroid = array([sumx / len(arc), sumy / len(arc)])

        # if len(front) > 0:
        #     for arc in front:
        #         sumx = 0
        #         sumy = 0
        #         delete_num = 0
        #         for arc_i in range(len(arc)):
        #             if is_point_inside_polygon(arc[arc_i], area_points_inside):
        #                 proceeded_frontiers.append(arc[arc_i])
        #                 sumx += arc[arc_i][0]
        #                 sumy += arc[arc_i][1]
        #             else:
        #                 delete_num += 1
        #         if len(arc) > delete_num:
        #             if len(arc) - delete_num >= 8: 
        #                 cms = MeanShift(bandwidth = 3.5 )
        #                 cms.fit(arc)
        #                 new_centroid = cms.cluster_centers_
        #             else:
        #                 new_centroid = array([sumx / (len(arc) - delete_num), sumy / (len(arc)  - delete_num)])

                # print("new_centroid",new_centroid)
                # print("shape",len(new_centroid.shape))
                # print("len",len(new_centroid))
                # if test == 1:
                #     print("used cluster")
                # else:
                #     print("used average")

                if len(centroids) == 0 and len(new_centroid.shape) == 1:
                    centroids = new_centroid.reshape(1, -1)
                elif len(centroids) == 0 and len(new_centroid.shape) > 1:
                    centroids = new_centroid
                else:
                    centroids = vstack((centroids, new_centroid))

        row_delete = []
        if len(centroids) > 0:
            for j in range(len(centroids)):
                LineEnd_w = []
                LineEnd_g = []
                LineEnd_w.append(centroids[j])
                LineEnd_w.append(now_odom_array)
                
                for end_w in LineEnd_w:                
                    LineEnd_g.append(GridCoordinate(mapData, end_w))
                world_line, line = BresenhamLine(mapData, LineEnd_g[0][0], LineEnd_g[0][1], LineEnd_g[1][0], LineEnd_g[1][1])

                if len(all_bresenham) == 0:
                    all_bresenham = world_line
                else:
                    all_bresenham = all_bresenham + world_line                        
                
                if not GridValueList(mapData, line) or not is_point_inside_polygon(centroids[j], area_points_inside):
                    row_delete.append(j)

            adjust = 0
            for r in row_delete:  
                centroids = delete(centroids, r - adjust, axis=0)
                adjust += 1

        # if len(centroids) == 0:
        #     rospy.set_param('~/'+name+'/need_change_radius','yes')
        #     radius_changing = True
        #     add_new = False
        # else:
        #     rospy.set_param('~/'+name+'/need_change_radius','no')
        #     radius_changing = False       

        if len(all_path)>0:
            path_record = [now_odom_array]
            pp_dis = linalg.norm(path_record - all_path[-1])
            if pp_dis >= 0.75:
                all_path = vstack((all_path, path_record))
        else:
            all_path = [now_odom_array]

        if not radius_changing:
            if len(all_path) > 0:
                row_delete = []
                for i in range(len(centroids)):
                    if i < len(centroids):
                        distance = linalg.norm(all_path - centroids[i],  axis=1)
                        if any(distance < 4):
                            row_delete.append(i)

                adjust = 0
                for r in row_delete:  
                    centroids = delete(centroids, r - adjust, axis=0)
                    adjust += 1

        row_delete = []
        for i in range(len(centroids)):
            if i < len(centroids):
                close = too_close_to_border(centroids[i], area_points_inside)
                if close:
                    row_delete.append(i)
    
        adjust = 0
        for r in row_delete:  
            centroids = delete(centroids, r - adjust, axis=0)
            adjust += 1
        
        
        delete_radius = 4
        if not radius_changing:
            row_delete = []
            for i in range(len(centroids)):
                if i < len(centroids):
                    distance = linalg.norm(all_path - centroids[i],  axis=1)
                    if any(distance < delete_radius):
                        row_delete.append(i)
        
            adjust = 0
            for r in row_delete:  
                centroids = delete(centroids, r - adjust, axis=0)
                adjust += 1 
# -------------------------------------------------------------------------
# publishing
        arraypoints.points = []
        path_points.points = []
        for d in all_path:
            path_p.x = d[0]
            path_p.y = d[1]
            path_points.points.append(copy(path_p))
        see_path.publish(path_points)

        bresenham_points.points = []
        for d in all_bresenham:
            bresenham_p.x = d[0]
            bresenham_p.y = d[1]
            bresenham_points.points.append(copy(bresenham_p))
        # print(bresenham_points.points)
        see_bresenham.publish(bresenham_points)
        
        for i in centroids:
            tempPoint.x = i[0]
            tempPoint.y = i[1]
            arraypoints.points.append(copy(tempPoint))
        filterpub.publish(arraypoints)

        pp = []
        for m in range(0, len(centroids)):
            cen.x = centroids[m][0]
            cen.y = centroids[m][1]
            pp.append(copy(cen))
        centroid_marker.points = pp
        pub3.publish(centroid_marker)

        pp = []
                         
        for q in range(0, len(proceeded_frontiers)):
            if q < len(proceeded_frontiers):
                p.x = proceeded_frontiers[q][0]
                p.y = proceeded_frontiers[q][1]
                pp.append(copy(p))
        points.points = pp
        pub.publish(points)
    
        quadrilateral_pub.publish(quadrilateral_msg)
        quadrilateral_pub_2.publish(quadrilateral_msg_2)

        # rate.sleep()
# -------------------------------------------------------------------------

if __name__ == '__main__':
    try:
        area = GetArea()
        if area.points_received == True:
            rospy.loginfo('Exploration area set.')
            node()
    except rospy.ROSInterruptException:
        pass
