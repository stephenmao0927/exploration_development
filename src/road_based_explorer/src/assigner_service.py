#!/usr/bin/env python
#coding: utf-8 
# created and maintained by Stephen Mao (maok0002@e.ntu.edu.sg)


#--------Include modules---------------
import rospy
from geometry_msgs.msg import Point
from geometry_msgs.msg import PolygonStamped
from nav_msgs.msg import OccupancyGrid
from road_based_explorer.srv import transfer_goal
from time import time
from functions_service import robot, BresenhamLine, GridValueList, GridCoordinate, GridValuePoint
from numpy.linalg import norm
from numpy import array, array_equal
from transforms3d.euler import quat2euler
from visualization_msgs.msg import Marker
from actionlib_msgs.msg import GoalStatusArray

def filter_with_distance(cur_po, new_po, cur_odom, new_odom):
	if cur_po == None:
		cur_po = new_po
		cur_odom = new_odom
	else:
		for i in range(0, len(new_po)):
			for p in cur_po:
				distance = norm(new_po[i] - p)
				if distance < 3:
					break
			cur_po.append(new_po[i])
			cur_odom.append(new_odom[i])

	return cur_po, cur_odom

def yaw_proceed(orientation):
    _, _, yaw = quat2euler(
            [orientation.w,
            orientation.x,
            orientation.y,
            orientation.z])
    return yaw

def calculate_location_cost(goal, corner):
	final_dis = None
	for corner_point in corner:
		new_dis = norm(array([corner_point.x,corner_point.y]) - goal)
		if not final_dis:
			final_dis = new_dis
		elif final_dis > new_dis:
			final_dis = new_dis

	return final_dis

def whether_require_goal(current_goal):
	rospy.logwarn("checking " + robot_name + "'s status")
	robot_reached = False
	status = rospy.wait_for_message('/'+ robot_name +'/move_base/status', GoalStatusArray)
	if len(status.status_list) == 1 and status.status_list[0].status == 3:
			robot_reached = True
	if current_goal is None:
		return True
	elif norm(robots[0].getPosition() - current_goal) <= 0.5 or robot_reached:
		# rospy.logwarn('robo pose is '+ str(robots[0].getPosition()))
		# rospy.logwarn('current goal is '+ str(current_goal))
		return True
	else:
		return False

# def is_stuck():
# 	status = rospy.wait_for_message('/'+robot_name+'/move_base/status', GoalStatusArray)
# 	for element_status in status.status_list:
# 		if element_status.status == 1:
# 			current_goal_id = element_status.goal_id.stamp.secs
# 			robot_state = 0 # 0 means the robot has not reached the current goal
# 			break
# 		elif element_status.goal_id.stamp.secs == current_goal_id and element_status.status == 3:
# 			robot_state = 1 # 1 means the robot has reached the current goal

# 	update_position = robots[0].getPosition()
# 	dis = norm(update_position - history_position)

# 	if dis < 0.001 and robot_state == 0:
# 		stuck_count += 1
# 	elif robot_state == 1:
# 		stuck_count = 0

# 	history_position = update_position

# 	if stuck_count > 10:
# 		return True
# 	else:
# 		return False

def client_call(need_goal):
	try:	
		rospy.wait_for_service('transfer_goal')
		ask_goal = rospy.ServiceProxy('transfer_goal', transfer_goal)
		srv_filter = ask_goal(need_goal)
		return srv_filter.goals, srv_filter.positions
	
	except rospy.ServiceException as e:
    	    rospy.logerr("Service call failed: %s", e)

def filter_with_update_map(goal_list, position_list):
	map = []
	delete_index = []
	map = rospy.wait_for_message(map_topic, OccupancyGrid)
	for i in range(0,len(goal_list)):
		LineEnd_w = []
		LineEnd_g = []
		LineEnd_w.append(goal_list[i])
		LineEnd_w.append(position_list[i])
		
		for end_w in LineEnd_w:                
			LineEnd_g.append(GridCoordinate(map, end_w))
		__, line = BresenhamLine(map, LineEnd_g[0][0], LineEnd_g[0][1], LineEnd_g[1][0], LineEnd_g[1][1])
		if not GridValueList(map, line) or not GridValuePoint(map, GridCoordinate(map, goal_list[i])):
			delete_index.append(i)
			rospy.logwarn('need delete')
	
	goal_list = [arr for idx, arr in enumerate(goal_list) if idx not in delete_index]
	position_list = [arr for idx, arr in enumerate(position_list) if idx not in delete_index]
	print(goal_list)
	print(position_list)

	return goal_list, position_list
	
#-------------------------------------------------------------------------			
# assigner goals for navigation
def assign(goal_list, position_list):	
#-------------------------------------------------------------------------			
#looking for the best goal to navigate to
	revenue_record=[]
	centroid_record=[]
	new_goal = None

	for ip in range(0,len(goal_list)):
		dis_cost = norm(robots[0].getPosition()-goal_list[ip])	
		location_cost = calculate_location_cost(goal_list[ip], corner)
		revenue =  999 - dis_cost * 0.8 - location_cost * 0.2
		revenue_record.append(revenue)
		centroid_record.append(goal_list[ip])

#-------------------------------------------------------------------------
# send the bset goal and log info	
	if len(goal_list) > 0:
		winner_id=revenue_record.index(max(revenue_record))
		robots[0].sendGoal(centroid_record[winner_id])
		new_goal = centroid_record[winner_id]
		rospy.loginfo( robot_name +"  assigned to  " + str(centroid_record[winner_id]))
		
		goal_list.pop(winner_id)
		position_list.pop(winner_id)
		rospy.sleep(delay_after_assignement)

	return new_goal, goal_list, position_list


# Node----------------------------------------------

def node():
	global robot_name, corner, delay_after_assignement, robots, n_robots, map_topic
	rospy.init_node('assigner', anonymous=False)

#---------------------------------------------------------------------------------------------------------------
# fetching all parameters
	map_topic = rospy.get_param('assigner/map_topic','')
	print('map_topic: ', map_topic)
	robot_name = rospy.get_param('assigner/robot_name','')
	print('robot_name: ', robot_name)
	map_topic = rospy.get_param('assigner/map_topic','')
	delay_after_assignement = rospy.get_param('assigner/delay_after_assignement',0.5)
	rateHz = rospy.get_param('assigner/rate',5)	
	n_robots = rospy.get_param('n_robots',1)
	rate = rospy.Rate(rateHz)
	
	corner_raw = rospy.wait_for_message('quadrilateral_inside', PolygonStamped)
	corner = corner_raw.polygon.points
	pub_goal_list = rospy.Publisher('goal_list_saved', Marker, queue_size=3)
	
#---------------------------------------------------------------------------------------------------------------
# initialize robot
	robots = []
	a = robot(robot_name)
	robots.append(a)

	# for i in range(0,n_robots):
	# 	robots[i].sendGoal(robots[i].getPosition())

	# current_goal = robots[0].getPosition()
	# history_position = robots[0].getPosition()
	mapData = rospy.wait_for_message(map_topic, OccupancyGrid)

	goal_saved = Point()
	goal_saved.z = 0
	
	show_goal_saved = Marker()
	show_goal_saved.header.frame_id = "/" + mapData.header.frame_id
	show_goal_saved.header.stamp = rospy.Time.now()
	show_goal_saved.ns = "markers3"
	show_goal_saved.id = 4
	show_goal_saved.type = Marker.POINTS
	show_goal_saved.action = Marker.ADD
	show_goal_saved.pose.orientation.w = 1.0
	show_goal_saved.scale.x = 0.2
	show_goal_saved.scale.y = 0.2
	show_goal_saved.color.r = 255.0/255.0
	show_goal_saved.color.g = 0.0/255.0
	show_goal_saved.color.b = 0.0/255.0
	show_goal_saved.color.a = 1
	show_goal_saved.lifetime = rospy.Duration()
	
	current_goal = None
	goal_list = []
	position_list = []

#-------------------------------------------------------------------------
#---------------------     Main   Loop     -------------------------------
#-------------------------------------------------------------------------
	while not rospy.is_shutdown():	 
		count = 0
		new_goals = []
		array_new_goals = []
		array_new_positions = []
		need_goal = whether_require_goal(current_goal)

		if need_goal:
			while count < 3:
				count += 1
				new_goals, new_positions = client_call(need_goal)
				if new_goals:
					rospy.logwarn("goal received for " + robot_name)
					for point in new_goals.points:
						array_new_goals.append(array([point.x,point.y]))
					for point in new_positions.points:
						array_new_positions.append(array([point.x,point.y]))
					goal_list, position_list= filter_with_distance(goal_list, array_new_goals, position_list, array_new_positions)
					break
				else:
					rate.sleep()

			if not new_goals:	
				rospy.logwarn("no nearby goal found for " + robot_name)
			
			# print(array_new_goals)

			goal_list, position_list = filter_with_update_map(goal_list, position_list)
			current_goal, goal_list, position_list = assign(goal_list, position_list)

			goal_point_list = []
			for element in goal_list:
				goal_saved.x = element[0]
				goal_saved.y = element[1]
				goal_point_list.append(goal_saved)
			show_goal_saved.points = goal_point_list
			pub_goal_list.publish(show_goal_saved)

		rate.sleep()
#-------------------------------------------------------------------------

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
 
 
 
 
