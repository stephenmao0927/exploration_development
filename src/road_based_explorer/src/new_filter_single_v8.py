#!/usr/bin/env python
# created and maintained by Stephen Mao (maok0002@e.ntu.edu.sg)

# --------Include modules---------------
from copy import copy
import rospy
import numpy as np
import math
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PointStamped, PolygonStamped
import tf
from numpy import array, vstack, fromstring, array_equal, delete, round, linalg
from sklearn.cluster import MeanShift
from rrt_exploration.msg import RoboArcArray,PointArray
from actionlib_msgs.msg import GoalStatusArray
import ast
import matplotlib.pyplot as plt
from numpy.linalg import norm


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
all_centroid = []
frontiers = []
all_path = []
area_points = []
area_points_inside = []
odom_record = []
proceeded_frontiers = []

class RealTimePlotter():
    def __init__(self):
        self.times = []  
        self.loops = []  

        plt.ion()
        self.figure, self.ax = plt.subplots()
        self.line, = self.ax.plot(self.loops, self.times)
        self.ax.set_xlabel('Loop')
        self.ax.set_ylabel('Time')
        self.ax.set_title('Time of filter loop for '+robot_name)

    def update_plot(self, time, loop):
        self.times.append(time)
        self.loops.append(loop)

        self.line.set_xdata(self.loops)
        self.line.set_ydata(self.times)
        self.ax.relim()
        self.ax.autoscale_view()
        self.figure.canvas.flush_events()

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

def BresenhamLine(x1, y1, x2, y2):
    line = []
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
    WorldCoordinate(mapData, line)
    return line

def GridCoordinate(mapData, p):
    ''' world coordinate to grid coordinate '''
    resolution = mapData.info.resolution
    startx = mapData.info.origin.position.x
    starty = mapData.info.origin.position.y
    gridplace = (int(np.floor((p[0]-startx)/resolution)),
            int(np.floor((p[1]-starty)/resolution)))
    return gridplace

def WorldCoordinate(mapData, list):
    global all_bresenham
    ''' grid coordinate to world coordinate '''
    resolution = mapData.info.resolution
    x = mapData.info.origin.position.x
    y = mapData.info.origin.position.y

    for i in range(len(list)):
        all_bresenham.append((resolution*list[i][0]+ x, resolution*list[i][1]+ y))

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

def GridValueList(mapData, list):
        ''' grid coordinate to grid value '''
        width = mapData.info.width
        height = mapData.info.height
        Data = mapData.data
        newlist = []
        for i in range(len(list)):
            index = list[i][1]*width + list[i][0]
            if Data[int(index)] == 0:
                continue
            else:
                return False
        return True

def GridValuePoint(mapData, p):
        ''' grid coordinate to grid value '''
        width = mapData.info.width
        height = mapData.info.height
        Data = mapData.data
        index = p[1]*width + p[0]
        if Data[int(index)] == 0:
            return True
        else:
            return False
        

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

# def goal_status_callback(msg):
#     global robot_state
#     for status in msg.status_list:
#         if status.status == status.ACTIVE:
#             robot_state = 0
#             return

#     robot_state = 1

# Node----------------------------------------------


def node():
    global all_bresenham, all_path, frontiers, mapData, n_robots, ready_for_goal, name, all_centroid, odom_record, area_points, area_points_inside, namespace_init_count, radius_changing
    name = None

    rate = rospy.Rate(rateHz)
# -------------------------------------------
    rospy.Subscriber(map_topic, OccupancyGrid, mapCallBack)
    
# ---------------------------------------------------------------------------------------------------------------
# wait if map is not received yet
    while (len(mapData.data) < 1):
        rospy.loginfo('Waiting for the map')
        rospy.sleep(0.1)
        pass

    global_frame = "/"+mapData.header.frame_id

    tfLisn = tf.TransformListener()
    tfLisn.waitForTransform(
        global_frame[1:], '/'+robot_frame, rospy.Time(0), rospy.Duration(10.0))

    rospy.Subscriber(goals_topic, RoboArcArray, callback=callBack,
                     callback_args=[tfLisn, global_frame[1:]])
    
    while(not name):
            # print('waiting for name')
            continue
    
    # rospy.Subscriber('/'+name+'/move_base/status', GoalStatusArray, goal_status_callback)
    pub = rospy.Publisher('frontiers', Marker, queue_size=10)
    pub2 = rospy.Publisher('updating_centroids', Marker, queue_size=10)
    pub3 = rospy.Publisher('centroids', Marker, queue_size=10)
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
    points_clust = Marker()
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

    centroid_marker.color.r = 255.0/255.0
    centroid_marker.color.g = 0.0/255.0
    centroid_marker.color.b = 0.0/255.00

    centroid_marker.color.a = 1
    centroid_marker.lifetime = rospy.Duration()

    cen = Point()
    cen.z = 0

# Set the frame ID and timestamp.  See the TF tutorials for information on these.
    points.header.frame_id = mapData.header.frame_id
    points.header.stamp = rospy.Time.now()

    points.ns = "markers2"
    points.id = 0

    points.type = Marker.POINTS

# Set the marker action for latched frontiers.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
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

    points_clust.header.frame_id = mapData.header.frame_id
    points_clust.header.stamp = rospy.Time.now()

    points_clust.ns = "markers3"
    points_clust.id = 4

    points_clust.type = Marker.POINTS

# Set the marker action for centroids.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    points_clust.action = Marker.ADD

    points_clust.pose.orientation.w = 1.0

    points_clust.scale.x = 0.2
    points_clust.scale.y = 0.2
    points_clust.color.r = 0.0/255.0
    points_clust.color.g = 255.0/255.0
    points_clust.color.b = 0.0/255.0

    points_clust.color.a = 1
    points_clust.lifetime = rospy.Duration()

    temppoint = PointStamped()
    temppoint.header.frame_id = mapData.header.frame_id
    temppoint.header.stamp = rospy.Time(0)
    temppoint.point.z = 0.0

    arraypoints = PointArray()
    tempPoint = Point()
    tempPoint.z = 0.0

    # plot = RealTimePlotter()
    no_add_loop = 0
    goal_cur_dis = 0
    goal_array = ''
    add_new = False
    current_goal_id = 0
    # rospy.set_param('~/'+name+'/get_goal_status', 'ready')
# -------------------------------------------------------------------------
# ---------------------     Main   Loop     -------------------------------
# -------------------------------------------------------------------------
    while not rospy.is_shutdown():
        # -------------------------------------------------------------------------
        proceeded_frontiers = []
        centroids = []
        all_bresenham = []
        front = copy(frontiers)

        goal_str = rospy.get_param('~/'+name+'/goal','')

        if len(front) > 0:
            for arc in front:
                sumx = 0
                sumy = 0
                delete_num = 0
                for arc_i in range(len(arc)):
                    if is_point_inside_polygon(arc[arc_i], area_points_inside):
                        proceeded_frontiers.append(arc[arc_i])
                        sumx += arc[arc_i][0]
                        sumy += arc[arc_i][1]
                    else:
                        delete_num += 1
                if len(arc) > delete_num:
                    if len(arc) - delete_num >= 8:
                        cms = MeanShift(bandwidth = 3.5 )
                        cms.fit(arc)
                        new_centroid = cms.cluster_centers_
                    else:
                        new_centroid = array([sumx / (len(arc) - delete_num), sumy / (len(arc)  - delete_num)])


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

        # print("raw centroid number",len(centroids))
        # ready_for_goal = rospy.get_param('~/'+name+'/get_goal_status', 'ready')

        odom = rospy.wait_for_message('/'+name+'/Odometry', Odometry)
        now_odom_array = array([odom.pose.pose.position.x,odom.pose.pose.position.y])

        row_delete = []
        if len(centroids) > 0:
            for j in range(len(centroids)):
                LineEnd_w = []
                LineEnd_g = []
                LineEnd_w.append(centroids[j])
                LineEnd_w.append(now_odom_array)
                
                for end_w in LineEnd_w:                
                    LineEnd_g.append(GridCoordinate(mapData, end_w))
                line = BresenhamLine(LineEnd_g[0][0], LineEnd_g[0][1], LineEnd_g[1][0], LineEnd_g[1][1])
                if not GridValueList(mapData, line) or not is_point_inside_polygon(centroids[j], area_points_inside):
                    row_delete.append(j)

            adjust = 0
            for r in row_delete:  
                centroids = delete(centroids, r - adjust, axis=0)
                adjust += 1 

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

        if not goal_str:
            robot_state = 2 # 2 means there's no goal for the robot
        else:
            status = rospy.wait_for_message('/'+ name +'/move_base/status', GoalStatusArray)
            for element_status in status.status_list:
                if element_status.status == 1:
                    current_goal_id = element_status.goal_id.stamp.secs
                    robot_state = 0 # 0 means the robot has not reached the current goal
                    stuck = rospy.get_param('~/'+name+'/is_stuck')
                    if stuck == "true":
                        robot_state = 4 # 4 means the robot has not reached the current goal but it's stuck
                    break
                elif element_status.goal_id.stamp.secs == current_goal_id and element_status.status == 3:
                    robot_state = 1 # 1 means the robot has reached the current goal

        # print("centroid number after bresenham",len(centroids))  
        # print("current centroid number for " + name,len(centroids))


        if len(centroids) == 0:
            rospy.set_param('~/'+name+'/need_change_radius','yes')
            radius_changing = True
            add_new = False
        else:
            rospy.set_param('~/'+name+'/need_change_radius','no')
            radius_changing = False         

        if len(all_path)>0:
            path_record = [now_odom_array]
            pp_dis = linalg.norm(path_record - all_path[-1])
            if pp_dis >= 0.75:
                all_path = vstack((all_path, path_record))
        else:
            all_path = [now_odom_array]

        exploration_num_b = len(all_centroid)

        if len(centroids) == 1 and robot_state == 0: 
            add_new = False
        
        # if len(centroids) == 1 and robot_state == 0 and ready_for_goal == 'ready': 
        #     # if name == "robot_1":
        #     #     rospy.loginfo(name+' one centroid, ready, with goal')
        #     if len(all_centroid) > 0 and centroids not in all_centroid:
        #         add = True
        #         for existed in all_centroid:
        #             if linalg.norm(centroids[0] - existed) < 4:
        #                 add = False
        #                 add_new = False
        #         if add:
        #             all_centroid = vstack((all_centroid, centroids[0]))
        #             odom_record = vstack((odom_record, now_odom_array))
        #             add_new = True
        #     elif len(all_centroid) == 0:
        #         all_centroid = centroids
        #         odom_record = now_odom_array.reshape(1, -1)
        #         add_new = True
        #     # rospy.set_param('~/'+name+'/get_goal_status','not')   

        if robot_state == 2:
                all_centroid = centroids
                odom_record = now_odom_array.reshape(1, -1)

        if len(centroids) == 1 and (robot_state == 1 or robot_state == 4):
            if len(all_centroid) > 0 and centroids[0] not in all_centroid:
                add = True
                for existed in all_centroid:
                    if linalg.norm(centroids[0] - existed) < 4:
                        add = False
                if add:
                    all_centroid = vstack((all_centroid, centroids[0]))
                    odom_record = vstack((odom_record, now_odom_array))
            elif len(all_centroid) == 0:
                all_centroid = centroids
                odom_record = now_odom_array.reshape(1, -1)

            # rospy.set_param('~/'+name+'/get_goal_status','not')    
        
        elif len(centroids) > 1:
            # cms = MeanShift(bandwidth = 5 )
            # cms.fit(centroids)
            # centroids = cms.cluster_centers_
            # rospy.loginfo(name+' multiple centroids')
            if len(all_centroid)>0:
                for m in centroids:
                    add = True
                    for existed in all_centroid:
                        if linalg.norm(m - existed) < 4:
                            add = False
                    if add:
                        all_centroid = vstack((all_centroid, m))
                        odom_record = vstack((odom_record, now_odom_array))

            else:
                all_centroid = centroids
                add_odom = 0
                odom_record = now_odom_array.reshape(1, -1)
                while add_odom < len(centroids) - 1:
                    odom_record = vstack((odom_record, now_odom_array))
                    add_odom += 1

            # rospy.set_param('~/'+name+'/get_goal_status','not')
        
        exploration_num_a = len(all_centroid)

        if exploration_num_b < exploration_num_a:
            add_new = True
            no_add_loop = 0
        else:
            add_new = False

        if not add_new and robot_state == 1:
            no_add_loop += 1

        # print(robot_name + ' no add loop', no_add_loop)
        # if robot_state == 1:
        #     print(robot_name + ' goal reached')
        # else:
        #     print(robot_name + ' goal not reached')

        # if len(all_centroid)>1:
        #     old_num = len(all_centroid)
        #     cms = MeanShift(bandwidth = 4.5 )
        #     cms.fit(all_centroid)
        #     all_centroid = cms.cluster_centers_
        #     print("decresed due to clustering",old_num - len(all_centroid))

        # print("total after selecting procedure",len(all_centroid))

# -------------------------------------------------------------------------
# clearing centroids which are useless and already set as goals
#         
        if len(goal_str)>0 and len(all_centroid)>0:
            delete_array = fromstring(goal_str.strip('[]'), sep=' ')
            for row in range(len(all_centroid)):
                if row < len(all_centroid):
                    if array_equal(delete_array, round(all_centroid[row], 8)):
                        all_centroid = delete(all_centroid, row, axis=0)
        
        # delete_radius = float(rospy.get_param('~/'+name+'/radius'))
        delete_radius = 4
        # rospy.loginfo('the current radius of deleting circle is'+str(delete_radius))

        row_delete = []
        for i in range(len(all_centroid)):
            if i < len(all_centroid) and i < len(odom_record):
                close = too_close_to_border(all_centroid[i], area_points_inside)
                if close:
                    row_delete.append(i)
    
        adjust = 0
        for r in row_delete:  
            all_centroid = delete(all_centroid, r - adjust, axis=0)
            odom_record = delete(odom_record, r - adjust, axis=0)
            adjust += 1 

        
        # print(len(all_centroid),len(odom_record))
        if not radius_changing:
            row_delete = []
            for i in range(len(all_centroid)):
                if i < len(all_centroid) and i < len(odom_record):
                    distance = linalg.norm(all_path - all_centroid[i],  axis=1)
                    if any(distance < delete_radius):
                        row_delete.append(i)
        
            adjust = 0
            for r in row_delete:  
                all_centroid = delete(all_centroid, r - adjust, axis=0)
                odom_record = delete(odom_record, r - adjust, axis=0)
                adjust += 1 

            row_delete = []
            for row in range(len(all_centroid)):
                if row < len(all_centroid) and row < len(odom_record):
                    grid_position = GridCoordinate(mapData, all_centroid[row])
                    if not GridValuePoint(mapData, grid_position):
                        row_delete.append(row)

            adjust = 0
            for r in row_delete:  
                all_centroid = delete(all_centroid, r - adjust, axis=0)
                odom_record = delete(odom_record, r - adjust, axis=0)
                adjust += 1 
  
        row_delete = []
        for row2 in range(len(all_centroid)):
            if row2 < len(all_centroid) and row2 < len(odom_record):
                LineEnd_w = []
                LineEnd_g = []
                LineEnd_w.append(all_centroid[row2])
                LineEnd_w.append(odom_record[row2])
                
                for end_w in LineEnd_w:                
                    LineEnd_g.append(GridCoordinate(mapData, end_w))
                line = BresenhamLine(LineEnd_g[0][0], LineEnd_g[0][1], LineEnd_g[1][0], LineEnd_g[1][1])
                if not GridValueList(mapData, line):
                    row_delete.append(row2)

        adjust = 0
        for r in row_delete:  
            all_centroid = delete(all_centroid, r - adjust, axis=0)
            odom_record = delete(odom_record, r - adjust, axis=0)
            adjust += 1

        # print("total after clearing procedure",len(all_centroid))
        # rospy.loginfo('')
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
        
        for i in all_centroid:
            tempPoint.x = i[0]
            tempPoint.y = i[1]
            arraypoints.points.append(copy(tempPoint))
        filterpub.publish(arraypoints)

        pp = []
        for m in range(0, len(all_centroid)):
            cen.x = all_centroid[m][0]
            cen.y = all_centroid[m][1]
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
        
        pp = []
        for r in range(0, len(centroids)):
            if r < len(centroids):
                c.x = centroids[r][0]
                c.y = centroids[r][1]
                pp.append(copy(c))
    
        points_clust.points = pp
        quadrilateral_pub.publish(quadrilateral_msg)
        quadrilateral_pub_2.publish(quadrilateral_msg_2)
        pub.publish(points)
        pub2.publish(points_clust)

        if ((add_new or no_add_loop > 20) and robot_state == 1) or robot_state == 2:
            # print('can send')
            rospy.set_param('~/'+name+'/ready_to_send','yes')
            no_add_loop = 0
        else:
            # print('cannot send')
            rospy.set_param('~/'+name+'/ready_to_send','no')

        rate.sleep()
# -------------------------------------------------------------------------

if __name__ == '__main__':
    try:
        area = GetArea()
        if area.points_received == True:
            rospy.loginfo('Exploration area set.')
            node()
    except rospy.ROSInterruptException:
        pass
