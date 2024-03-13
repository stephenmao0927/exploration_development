#!/usr/bin/env python

# --------Include modules---------------
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PointStamped, PolygonStamped
import ast

# fetching all parameters
robot_name = rospy.get_param('area_config/robot_name', '')
exploration_points = rospy.get_param('area_config/exploration_points', '')
print('exploration_points: ', exploration_points)
little_gain_distance = rospy.get_param('area_config/little_gain_distance', '')

area_points = []
area_points_inside = []

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

def publish_quadrilateral(decided_points):
    quadrilateral_msg = PolygonStamped()
    quadrilateral_msg.header.stamp = rospy.Time.now()
    quadrilateral_msg.header.frame_id = robot_name+'/map'
    quadrilateral_msg.polygon.points = decided_points
    return quadrilateral_msg

# Node----------------------------------------------

def node():
    quadrilateral_pub = rospy.Publisher('quadrilateral', PolygonStamped, queue_size=1)
    quadrilateral_pub_2 = rospy.Publisher('quadrilateral_inside', PolygonStamped, queue_size=1)
    quadrilateral_msg = publish_quadrilateral(area_points)
    quadrilateral_msg_2 = publish_quadrilateral(area_points_inside)
# -------------------------------------------------------------------------
# ---------------------     Main   Loop     -------------------------------
# -------------------------------------------------------------------------
    while not rospy.is_shutdown():
        # -------------------------------------------------------------------------
        quadrilateral_pub.publish(quadrilateral_msg)
        quadrilateral_pub_2.publish(quadrilateral_msg_2)
# -------------------------------------------------------------------------

if __name__ == '__main__':
    try:
        area = GetArea()
        if area.points_received == True:
            rospy.loginfo('Exploration area set.')
            node()
    except rospy.ROSInterruptException:
        pass
