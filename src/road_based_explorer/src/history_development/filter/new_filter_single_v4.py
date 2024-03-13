#!/usr/bin/env python

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
from functions import gridValue, informationGain
from sklearn.cluster import MeanShift
from rrt_exploration.msg import RoboArcArray,PointArray
from actionlib_msgs.msg import GoalStatusArray


# fetching all parameters
map_topic = rospy.get_param('map_topic', 'robot_1/move_base_node/global_costmap/costmap')
threshold = rospy.get_param('ignoring_threshold', 0.5)
info_radius = rospy.get_param('info_radius', 10.0)
goals_topic = rospy.get_param('goals_topic', '/detected_points')
n_robots = rospy.get_param('n_robots', 1)
namespace = rospy.get_param('namespace', '')
namespace_init_count = rospy.get_param('namespace_init_count', 1)
rateHz = rospy.get_param('rate', 100)
global_costmap_topic = rospy.get_param(
    'global_costmap_topic', 'map')
robot_frame = rospy.get_param('robot_frame', 'robot_1/map')
robot_name = rospy.get_param('robot_name', 'robot_1')


mapData = OccupancyGrid()
ready_for_goal = None
robot_state = None
loop = 0
all_centroid = []
frontiers = []
globalmaps = []
all_path = []
area_points = []
area_points_inside = []

class GetArea(object):
    global area_points, area_points_inside
    def __init__(self):
        rospy.init_node('filter', anonymous=True)
        rospy.loginfo("Please make sure that the points are published in clockwise order or in counterclockwise order!")
        self.points_received = False

        while not self.points_received:
            rospy.Subscriber('/clicked_point', PointStamped, self.point_callback)
        
        self.get_inside_quadrilateral()

    def point_callback(self, point_msg):
        if point_msg.point not in area_points:
            area_points.append(point_msg.point)

        if len(area_points) == 4:
            self.points_received = True

    def get_inside_quadrilateral(self):

        A1, B1, C1 = self.line_from_two_points(area_points[0].x, area_points[0].y, area_points[2].x, area_points[2].y)
        A2, B2, C2 = self.line_from_two_points(area_points[1].x, area_points[1].y, area_points[3].x, area_points[3].y)
        center = self.intersection_of_lines(A1, B1, C1, A2, B2, C2)


        A1, B1, C1 = self.line_from_two_points(area_points[0].x, area_points[0].y, area_points[1].x, area_points[1].y)
        A2, B2, C2 = self.line_from_two_points(area_points[1].x, area_points[1].y, area_points[2].x, area_points[2].y)
        A3, B3, C3 = self.line_from_two_points(area_points[2].x, area_points[2].y, area_points[3].x, area_points[3].y)
        A4, B4, C4 = self.line_from_two_points(area_points[0].x, area_points[0].y, area_points[3].x, area_points[3].y)

        C11,C12 = self.find_parallel_lines_with_distance(A1, B1, C1, 4)
        if self.point_to_line_distance(A1, B1, C11, center[0], center[1]) >= self.point_to_line_distance(A1, B1, C12, center[0], center[1]):
            C1 = C12
        else:
            C1 = C11

        C21,C22 = self.find_parallel_lines_with_distance(A2, B2, C2, 4)
        if self.point_to_line_distance(A2, B2, C21, center[0], center[1]) >= self.point_to_line_distance(A2, B2, C22, center[0], center[1]):
            C2 = C22
        else:
            C2 = C21

        C31,C32 = self.find_parallel_lines_with_distance(A3, B3, C3, 4)
        if self.point_to_line_distance(A3, B3, C31, center[0], center[1]) >= self.point_to_line_distance(A3, B3, C32, center[0], center[1]):
            C3 = C32
        else:
            C3 = C31

        C41,C42 = self.find_parallel_lines_with_distance(A4, B4, C4, 4)
        if self.point_to_line_distance(A4, B4, C41, center[0], center[1]) >= self.point_to_line_distance(A4, B4, C42, center[0], center[1]):
            C4 = C42
        else:
            C4 = C41

            corner_point_inside = Point()
            corner_point_inside.z = 0
            corner_point_coordinate = self.intersection_of_lines(A1, B1, C1, A2, B2, C2)
            corner_point_inside.x = corner_point_coordinate[0]
            corner_point_inside.y = corner_point_coordinate[1]
            area_points_inside.append(corner_point_inside)

            corner_point_inside = Point()
            corner_point_inside.z = 0
            corner_point_coordinate = self.intersection_of_lines(A3, B3, C3, A2, B2, C2)
            corner_point_inside.x = corner_point_coordinate[0]
            corner_point_inside.y = corner_point_coordinate[1]
            area_points_inside.append(corner_point_inside)

            corner_point_inside = Point()
            corner_point_inside.z = 0
            corner_point_coordinate = self.intersection_of_lines(A3, B3, C3, A4, B4, C4)
            corner_point_inside.x = corner_point_coordinate[0]
            corner_point_inside.y = corner_point_coordinate[1]
            area_points_inside.append(corner_point_inside)

            corner_point_inside = Point()
            corner_point_inside.z = 0
            corner_point_coordinate = self.intersection_of_lines(A1, B1, C1, A4, B4, C4)
            corner_point_inside.x = corner_point_coordinate[0]
            corner_point_inside.y = corner_point_coordinate[1]
            area_points_inside.append(corner_point_inside)

    def line_from_two_points(self, x1, y1, x2, y2):
        A = y2 - y1
        B = x1 - x2
        C = x2 * y1 - x1 * y2
        return A, B, C

    def intersection_of_lines(self, A1, B1, C1, A2, B2, C2):
        coeff_matrix = np.array([[A1, B1], [A2, B2]])
        const_vector = np.array([-C1, -C2])
    
        intersection_point = np.linalg.solve(coeff_matrix, const_vector)
        
        return intersection_point

    def find_parallel_lines_with_distance(self, A, B, C, distance):
        norm = math.sqrt(A**2 + B**2)

        offset1 = distance * norm
        offset2 = -distance * norm

        C1 = C + offset1
        C2 = C + offset2

        return C1, C2

    def point_to_line_distance(self, A, B, C, x0, y0):
        signed_distance = abs(A * x0 + B * y0 + C) / math.sqrt(A**2 + B**2)
        
        return signed_distance

def publish_quadrilateral(area_points):
    quadrilateral_msg = PolygonStamped()
    quadrilateral_msg.header.stamp = rospy.Time.now()
    quadrilateral_msg.header.frame_id = 'robot_1/map'
    quadrilateral_msg.polygon.points = area_points
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
    return line

def GridCoordinate(mapData, p):
        ''' world coordinate to grid coordinate '''
        resolution = mapData.info.resolution
        startx = mapData.info.origin.position.x
        starty = mapData.info.origin.position.y
        gridplace = (int(np.floor((p[0]-startx)/resolution)),
                int(np.floor((p[1]-starty)/resolution)))
        return gridplace

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

def globalMap(data):
    global global1, globalmaps, litraIndx, namespace_init_count, n_robots
    global1 = data
    if n_robots > 1:
        indx = int(data._connection_header['topic']
                   [litraIndx])-namespace_init_count
    elif n_robots == 1:
        indx = 0
    globalmaps[indx] = data

def GridValue(mapData, list):
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

def goal_status_callback(msg):
    global robot_state
    for status in msg.status_list:
        if status.status == status.ACTIVE:
            robot_state = 0
            return

    robot_state = 1

# Node----------------------------------------------


def node():
    global frontiers, mapData, global1, global2, global3, globalmaps, litraIndx, n_robots, ready_for_goal, namespace_init_count, name, all_centroid, area_points, area_points_inside
    name = None

    litraIndx = len(namespace)
    rate = rospy.Rate(rateHz)
# -------------------------------------------
    rospy.Subscriber(map_topic, OccupancyGrid, mapCallBack)
    
    
# ---------------------------------------------------------------------------------------------------------------

    for i in range(0, n_robots):
        globalmaps.append(OccupancyGrid())

    if len(namespace) > 0:
        for i in range(0, n_robots):
            rospy.Subscriber(namespace+str(i+namespace_init_count) +
                             global_costmap_topic, OccupancyGrid, globalMap)
    elif len(namespace) == 0:
        rospy.Subscriber(global_costmap_topic, OccupancyGrid, globalMap)
# wait if map is not received yet
    while (len(mapData.data) < 1):
        rospy.loginfo('Waiting for the map')
        rospy.sleep(0.1)
        pass
# wait if any of robots' global costmap map is not received yet
    for i in range(0, n_robots):
        while (len(globalmaps[i].data) < 1):
            rospy.loginfo('Waiting for the global costmap')
            rospy.sleep(0.1)
            pass

    global_frame = "/"+mapData.header.frame_id

    tfLisn = tf.TransformListener()
    if len(namespace) > 0:
        for i in range(0, n_robots):
            tfLisn.waitForTransform(global_frame[1:], namespace+str(
                i+namespace_init_count)+'/'+robot_frame, rospy.Time(0), rospy.Duration(10.0))
    elif len(namespace) == 0:
        tfLisn.waitForTransform(
            global_frame[1:], '/'+robot_frame, rospy.Time(0), rospy.Duration(10.0))

    rospy.Subscriber(goals_topic, RoboArcArray, callback=callBack,
                     callback_args=[tfLisn, global_frame[1:]])
    
    while(not name):
            # print('waiting for name')
            continue
    
    rospy.Subscriber('/'+name+'/move_base/status', GoalStatusArray, goal_status_callback)
    pub = rospy.Publisher('frontiers', Marker, queue_size=10)
    pub2 = rospy.Publisher('updating_centroids', Marker, queue_size=10)
    pub3 = rospy.Publisher('centroids', Marker, queue_size=10)
    see_path = rospy.Publisher('path_show', Marker, queue_size=10)
    filterpub = rospy.Publisher('filtered_points', PointArray, queue_size=10)
    quadrilateral_pub = rospy.Publisher('/quadrilateral', PolygonStamped, queue_size=1)
    quadrilateral_pub_2 = rospy.Publisher('/quadrilateral_inside', PolygonStamped, queue_size=1)
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

    rospy.set_param('~/'+name+'/get_goal_status', 'ready')
# -------------------------------------------------------------------------
# ---------------------     Main   Loop     -------------------------------
# -------------------------------------------------------------------------
    while not rospy.is_shutdown():
        global all_path,loop
        # -------------------------------------------------------------------------
        centroids = []
        front = copy(frontiers)
        if len(front) > 0:
            for arc in front:
                sumx = 0
                sumy = 0
                for arc_i in range(len(arc)):
                    sumx += arc[arc_i][0]
                    sumy += arc[arc_i][1]
                new_centroid = array([sumx/len(arc), sumy/len(arc)])
                if len(centroids) == 0:
                    centroids = new_centroid.reshape(1, -1)
                else:
                    centroids = vstack((centroids,new_centroid))

        print("raw centroid number",len(centroids))
        ready_for_goal = rospy.get_param('~/'+name+'/get_goal_status', 'ready')
        # ready_for_path = rospy.get_param('~/'+name+'/new_path_ready', 'not')

        # if ready_for_path == 'ready': 
        #     rospy.set_param('~/'+name+'/new_path_ready','not')
        #     path1 = rospy.wait_for_message('/'+name+'/move_base_node/DWAPlannerROS/local_plan', Path)
        #     for i in range(len(path1.poses)):
        #         all_path.append(array([path1.poses[i].pose.position.x,path1.poses[i].pose.position.y]))
        #     path2 = rospy.wait_for_message('/'+name+'/move_base_node/DWAPlannerROS/global_plan', Path)
        #     for i in range(len(path2.poses)):
        #         all_path.append(array([path2.poses[i].pose.position.x,path2.poses[i].pose.position.y]))
        #     rospy.loginfo(all_path)

        odom = rospy.wait_for_message('/'+name+'/Odometry', Odometry)

        row_delete = []
        if len(centroids) > 1:
            for j in range(len(centroids)):
                LineEnd_w = []
                LineEnd_g = []
                LineEnd_w.append(centroids[j])
                LineEnd_w.append(array([odom.pose.pose.position.x,odom.pose.pose.position.y]))
                
                for end_w in LineEnd_w:                
                    LineEnd_g.append(GridCoordinate(mapData, end_w))
                line = BresenhamLine(LineEnd_g[0][0], LineEnd_g[0][1], LineEnd_g[1][0], LineEnd_g[1][1])
                if not GridValue(mapData, line) or not is_point_inside_polygon(centroids[j],area_points_inside):
                    row_delete.append(j)

            adjust = 0
            for r in row_delete:  
                centroids = delete(centroids, r - adjust, axis=0)
                adjust += 1 
        elif len(centroids) > 0:
            if not is_point_inside_polygon(centroids[0],area_points_inside):
                centroids = []

        print("centroid number after bresenham",len(centroids))           

        if len(all_path)>0:
            path_record = [array([odom.pose.pose.position.x,odom.pose.pose.position.y])]
            pp_dis = linalg.norm(path_record - all_path[-1])
            if pp_dis >= 0.75:
                all_path = vstack((all_path, path_record))
        else:
            all_path = [array([odom.pose.pose.position.x,odom.pose.pose.position.y])]
        
        
        if len(centroids) == 1 and robot_state == 0 and ready_for_goal == 'not': 
            rospy.loginfo('one centroid, not ready, with goal')
            pass
        
        if len(centroids) == 1 and robot_state == 0 and ready_for_goal == 'ready': 
            rospy.loginfo('one centroid, ready, with goal')
            if len(all_centroid) > 0 and centroids not in all_centroid:
                all_centroid = vstack((all_centroid, centroids))
            elif len(all_centroid) == 0:
                all_centroid = centroids
            rospy.set_param('~/'+name+'/get_goal_status','not')   


        if len(centroids) == 1 and robot_state == 1:
            rospy.loginfo('one centroid, no goal') 
            if len(all_centroid) > 0 and centroids[0] not in all_centroid:
                all_centroid = vstack((all_centroid, centroids))
            elif len(all_centroid) == 0:
                all_centroid = centroids
            rospy.set_param('~/'+name+'/get_goal_status','not')    
        
        elif len(centroids) > 1:
            # cms = MeanShift(bandwidth = 5 )
            # cms.fit(centroids)
            # centroids = cms.cluster_centers_
            rospy.loginfo('multiple centroids')
            for m in centroids:
                add = True
                if len(all_centroid)>0:
                    for existed in all_centroid:
                        if linalg.norm(m - existed) < 4:
                            add = False
                    if add:
                        all_centroid = vstack((all_centroid, m))
                else:
                    all_centroid = centroids
            rospy.set_param('~/'+name+'/get_goal_status','not')

        # if len(all_centroid)>1:
        #     old_num = len(all_centroid)
        #     cms = MeanShift(bandwidth = 4.5 )
        #     cms.fit(all_centroid)
        #     all_centroid = cms.cluster_centers_
        #     print("decresed due to clustering",old_num - len(all_centroid))

        print("total after selecting procedure",len(all_centroid))


# -------------------------------------------------------------------------
# clearing centroids which are useless and already set as goals
#         
        delete_str = rospy.get_param('~/'+name+'/goal')
        if len(delete_str)>0 and len(all_centroid)>0:
            delete_array = fromstring(delete_str.strip('[]'), sep=' ')
            for row in range(len(all_centroid)):
                if row < len(all_centroid):
                    if array_equal(delete_array, round(all_centroid[row], 8)):
                        all_centroid = delete(all_centroid, row, axis=0)
        
        # delete_radius = float(rospy.get_param('~/'+name+'/radius'))
        delete_radius = 3.5
        # rospy.loginfo('the current radius of deleting circle is'+str(delete_radius))
        for i in range(len(all_centroid)):
            if i < len(all_centroid):
                distance = linalg.norm(all_path - all_centroid[i],  axis=1)
                if any(distance < delete_radius):
                        all_centroid = delete(all_centroid, i, axis=0)

        print("total after clearing procedure",len(all_centroid))
        rospy.loginfo('')

       
# -------------------------------------------------------------------------
# publishing

        arraypoints.points = []
        path_points.points = []
        for d in all_path:
            path_p.x = d[0]
            path_p.y = d[1]
            path_points.points.append(copy(path_p))
        see_path.publish(path_points)
        
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
        proceeded_frontiers = []
        for arc in frontiers:
            for arc_p_i in range(len(arc)):
                proceeded_frontiers.append(arc[arc_p_i])
            
        for q in range(0, len(proceeded_frontiers)):
            if q < len(proceeded_frontiers):
                p.x = proceeded_frontiers[q][0]
                p.y = proceeded_frontiers[q][1]
                pp.append(copy(p))
        points.points = pp
        
        pp = []
        for r in range(0, len(centroids)):
            if q < len(centroids):
                c.x = centroids[r][0]
                c.y = centroids[r][1]
                pp.append(copy(c))
    
        points_clust.points = pp
        quadrilateral_pub.publish(quadrilateral_msg)
        quadrilateral_pub_2.publish(quadrilateral_msg_2)
        pub.publish(points)
        pub2.publish(points_clust)
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