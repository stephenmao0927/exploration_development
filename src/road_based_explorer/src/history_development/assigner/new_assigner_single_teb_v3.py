#!/usr/bin/env python
#coding: utf-8 

#--------Include modules---------------
from copy import copy
import matplotlib.pyplot as plt
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Twist, PolygonStamped
import tf
from nav_msgs.msg import OccupancyGrid,Odometry
import tf
from rrt_exploration.msg import PointArray
from time import time
from numpy import array
from numpy import linalg as LA
from numpy import all as All
from numpy import abs
import math 
from hunter_exploration.src.road_based_explorer.src.functions_deploy import robot,informationGain,discount
from numpy.linalg import norm
from transforms3d.euler import quat2euler

class RealTimePlotter():
    def __init__(self):
        self.times = []  
        self.loops = []  

        plt.ion()
        self.figure, self.ax = plt.subplots()
        self.line, = self.ax.plot(self.loops, self.times)
        self.ax.set_xlabel('Loop')
        self.ax.set_ylabel('Time')
        self.ax.set_title('Time of assigner receiving msg')

    def update_plot(self, time, loop):
        self.times.append(time)
        self.loops.append(loop)

        self.line.set_xdata(self.loops)
        self.line.set_ydata(self.times)
        self.ax.relim()
        self.ax.autoscale_view()
        self.figure.canvas.flush_events()

# plot = RealTimePlotter()

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


# mapData=OccupancyGrid()
frontiers=[]
globalmaps=[]
ready_to_send = False

# Subscribers' callbacks------------------------------
def callBack(data):
	global frontiers, loop, n_robots, robots, robot_name, current_goal, corner, waiting_count, delay_after_assignement, ready_to_send
	
	frontiers=[]
	for point in data.points:
		frontiers.append(array([point.x,point.y]))

	centroids=copy(frontiers)		
	# if len(centroids)>0:
	# 	rospy.loginfo("centroids received")	
#-------------------------------------------------------------------------			
# #Get information gain for each frontier point
# 		infoGain=[]
# 		for ip in range(0,len(centroids)):
# 			infoGain.append(informationGain(mapData,[centroids[ip][0],centroids[ip][1]],info_radius))
#-------------------------------------------------------------------------			
#get number of available/busy robots
	na=[] #available robots
	nb=[] #busy robots
	for i in range(0,n_robots):
		if (robots[i].getState()==1):
			nb.append(i)
		else:
			na.append(i)	
	# rospy.loginfo("available robots: "+str(na))	
#------------------------------------------------------------------------- 
#get dicount and update informationGain
	# for i in nb+na:
	# 	infoGain=discount(mapData,robots[i].assigned_point,centroids,infoGain,info_radius)
#-------------------------------------------------------------------------            
	revenue_record=[]
	centroid_record=[]
	id_record=[]
	# plot.update_plot(end-start,loop)
	
	if len(na)<1:
		revenue_record=[]
		centroid_record=[]
		id_record=[]
		for ir in nb:
			dis = norm(robots[ir].getPosition() - current_goal)
			# odom = rospy.wait_for_message('/' + robot_name + '/ackermann_steering_controller/odom',Odometry)
			# vel = odom.twist.twist.linear.x
			# if vel < 0.01:
			# 	waiting_count += 1
			# 	print(robot_name+' waiting count  '+str(waiting_count))
			# if dis > 6.0 and check_far:
			# 	far_count += 1
			# if dis < 0.75 or far_count > 50 or waiting_count > 50:
			# if dis < 2.25:
			# 	rospy.set_param('~/'+ robot_name +'/get_goal_status','ready')
			# 	# rospy.sleep(0.05)
			# 	# rospy.loginfo(robot_name + " is looking for new goal")
			
			if dis < 0.6:
				for ip in range(0, len(centroids)):
					dis_cost = norm(robots[ir].getPosition() - centroids[ip])
					location_cost = calculate_location_cost(centroids[ip], corner)
					revenue = 999 - dis_cost * 0.8 - location_cost * 0.2
					revenue_record.append(revenue)
					centroid_record.append(centroids[ip])
					id_record.append(ir)

				# far_count = 0
				# check_far = False
				
			else:
				break
					
	else:
		for ir in na:
			for ip in range(0,len(centroids)):
				dis_cost = norm(robots[ir].getPosition()-centroids[ip])	
				location_cost = calculate_location_cost(centroids[ip], corner)
				revenue =  999 - dis_cost * 0.8 - location_cost * 0.2
				revenue_record.append(revenue)
				centroid_record.append(centroids[ip])
				id_record.append(ir)
	
	# rospy.loginfo("revenue record: "+str(revenue_record))	
	# rospy.loginfo("centroid record: "+str(centroid_record))	
	# rospy.loginfo("robot IDs record: "+str(id_record))	
	
#-------------------------------------------------------------------------	
	ready_param = rospy.get_param('~/'+robot_name+'/ready_to_send','')
	# print(robot_name + " ready to send ? ", ready_param)
	if ready_param == 'yes':
		ready_to_send= True
	if ready_param == 'no':
		ready_to_send= False
	
	# if robot_name == "robot_1":
	# 	print(robot_name + " ready to send ? ", ready_to_send)

	if len(id_record)>0 and ready_to_send:
		winner_id=revenue_record.index(max(revenue_record))
		# rospy.loginfo(max(revenue_record))
		# rospy.loginfo(revenue_record)
		robots[id_record[winner_id]].sendGoal(centroid_record[winner_id])
		ready_to_send = False
		# check_far = True
		# waiting_count = 0
		current_goal = centroid_record[winner_id]
		# current_robot = str(namespace_init_count+id_record[winner_id])
		# rospy.loginfo(namespace+str(namespace_init_count+id_record[winner_id])+"  assigned to  "+str(centroid_record[winner_id]))	
		rospy.loginfo( robot_name +"  assigned to  "+str(centroid_record[winner_id]))
		rospy.sleep(delay_after_assignement)
	
	if current_goal is not None:
		rospy.set_param('~/'+ robot_name +'/goal',str(current_goal))

# def mapCallBack(data):
#     global mapData
#     mapData=data

# Node----------------------------------------------

def node():
	global frontiers, mapData, globalmaps, current_goal, plot, loop, n_robots, robots, robot_name, corner, delay_after_assignement, ready_to_send
	rospy.init_node('assigner', anonymous=False)
		

	# fetching all parameters
	map_topic = rospy.get_param('assigner/map_topic','')
	print('map_topic: ', map_topic)
	frontiers_topic = rospy.get_param('assigner/frontiers_topic','')
	print('frontiers_topic: ', frontiers_topic)
	robot_name = rospy.get_param('assigner/robot_name','')
	print('robot_name: ', robot_name)
	delay_after_assignement = rospy.get_param('assigner/delay_after_assignement',0.5)
	rateHz = rospy.get_param('assigner/rate',5)
	
	n_robots = rospy.get_param('n_robots',1)
	namespace = rospy.get_param('namespace','')
	namespace_init_count = rospy.get_param('namespace_init_count',1)
	rate = rospy.Rate(rateHz)
#-------------------------------------------
	# rospy.Subscriber(map_topic, OccupancyGrid, mapCallBack)
	
#---------------------------------------------------------------------------------------------------------------
	raw_frontiers = rospy.wait_for_message(frontiers_topic, PointArray)
	for point in raw_frontiers.points:
		frontiers.append(array([point.x,point.y]))
# wait if no frontier is received yet 
	# start = rospy.get_time()
	while len(frontiers)<1:
		pass
	centroids=copy(frontiers)
	# end = rospy.get_time()
	
#wait if map is not received yet
	# while (len(mapData.data)<1):
	# 	pass

	robots=[]

	# if len(namespace)>0:
	# 	for i in range(0,n_robots):
	# 		robots.append(robot(namespace+str(i+namespace_init_count)))

	# elif len(namespace)==0:
	a = robot(robot_name)
	robots.append(a)
	
	for i in range(0,n_robots):
		# rospy.loginfo(robots[i].getPosition())
		robots[i].sendGoal(robots[i].getPosition())

	current_goal = robots[0].getPosition()

	corner_raw = rospy.wait_for_message('quadrilateral_inside', PolygonStamped)
	corner = corner_raw.polygon.points
	rospy.Subscriber(frontiers_topic, PointArray, callBack)

#-------------------------------------------------------------------------
#---------------------     Main   Loop     -------------------------------
#-------------------------------------------------------------------------
	while not rospy.is_shutdown():	

#------------------------------------------------------------------------- 
		rate.sleep()
#-------------------------------------------------------------------------

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
 
 
 
 
