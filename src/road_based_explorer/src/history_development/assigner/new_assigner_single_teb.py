#!/usr/bin/env python
#coding: utf-8 

#--------Include modules---------------
from copy import copy
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point,Twist
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

current_goal = None

def yaw_proceed(orientation):
    _, _, yaw = quat2euler(
            [orientation.w,
            orientation.x,
            orientation.y,
            orientation.z])
    return yaw

mapData=OccupancyGrid()
frontiers=[]
global1=OccupancyGrid()
global2=OccupancyGrid()
global3=OccupancyGrid()
globalmaps=[]

# Subscribers' callbacks------------------------------
def callBack(data):
	global frontiers
	frontiers=[]
	for point in data.points:
		frontiers.append(array([point.x,point.y]))

def mapCallBack(data):
    global mapData
    mapData=data

# Node----------------------------------------------

def node():
	global frontiers,mapData,global1,global2,global3,globalmaps,current_goal
	rospy.init_node('assigner', anonymous=False)
	
	# fetching all parameters
	map_topic = rospy.get_param('assigner/map_topic','')
	print('map_topic: ', map_topic)
	frontiers_topic = rospy.get_param('assigner/frontiers_topic','')
	print('frontiers_topic: ', frontiers_topic)
	robot_name = rospy.get_param('assigner/robot_name','')
	print('robot_name: ', robot_name)
	delay_after_assignement = rospy.get_param('assigner/delay_after_assignement',0.5)
	rateHz = rospy.get_param('assigner/rate',100)
	
	n_robots = rospy.get_param('n_robots',1)
	namespace = rospy.get_param('namespace','')
	namespace_init_count = rospy.get_param('namespace_init_count',1)
	rate = rospy.Rate(rateHz)
#-------------------------------------------
	rospy.Subscriber(map_topic, OccupancyGrid, mapCallBack)
	rospy.Subscriber(frontiers_topic, PointArray, callBack)
#---------------------------------------------------------------------------------------------------------------
		
# wait if no frontier is received yet 
	while len(frontiers)<1:
		pass
	centroids=copy(frontiers)	
#wait if map is not received yet
	while (len(mapData.data)<1):
		pass

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

	waiting_count = 0
	

#-------------------------------------------------------------------------
#---------------------     Main   Loop     -------------------------------
#-------------------------------------------------------------------------
	while not rospy.is_shutdown():	
		centroids=copy(frontiers)			
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
		
		if current_goal is not None:
			if len(na)<1:
				revenue_record=[]
				centroid_record=[]
				id_record=[]
				for ir in nb:
					dis = norm(robots[ir].getPosition() - current_goal)
					odom = rospy.wait_for_message('/' + robot_name + '/ackermann_steering_controller/odom',Odometry)
					vel = odom.twist.twist.linear.x
					if vel < 0.01:
						waiting_count += 1
						print(robot_name+' waiting count  '+str(waiting_count))
					if dis < 0.75 or waiting_count > 10:
						if not centroids:
							rospy.set_param('~/'+ robot_name +'/get_goal_status','ready')
							# rospy.sleep(0.05)
						# rospy.loginfo(robot_name + " is looking for new goal")

						waiting_count = 0
						for ip in range(0, len(centroids)):
							cost=norm(robots[ir].getPosition()-centroids[ip])	
							revenue = 999 - cost
							revenue_record.append(revenue)
							centroid_record.append(centroids[ip])
							id_record.append(ir)
					else:
						break
						
		else:
			frontiers = rospy.wait_for_message(frontiers_topic, PointArray)
			for point in frontiers.points:
				centroids.append(array([point.x,point.y]))

			for ir in na:

				for ip in range(0,len(centroids)):
					cost=norm(robots[ir].getPosition()-centroids[ip])	
					revenue = 999 - cost
					revenue_record.append(revenue)
					centroid_record.append(centroids[ip])
					id_record.append(ir)
		
		# rospy.loginfo("revenue record: "+str(revenue_record))	
		# rospy.loginfo("centroid record: "+str(centroid_record))	
		# rospy.loginfo("robot IDs record: "+str(id_record))	
		
#-------------------------------------------------------------------------	
		if (len(id_record)>0):
			winner_id=revenue_record.index(max(revenue_record))
			# rospy.loginfo(max(revenue_record))
			# rospy.loginfo(revenue_record)
			robots[id_record[winner_id]].sendGoal(centroid_record[winner_id])
			current_goal = centroid_record[winner_id]
			# current_robot = str(namespace_init_count+id_record[winner_id])
			# rospy.set_param('~/robot_'+current_robot+'/new_path_ready','ready')
			# rospy.loginfo(namespace+str(namespace_init_count+id_record[winner_id])+"  assigned to  "+str(centroid_record[winner_id]))	
			rospy.loginfo( robot_name +"  assigned to  "+str(centroid_record[winner_id]))
			rospy.sleep(delay_after_assignement)
		
		if current_goal is not None:
			rospy.set_param('~'+ robot_name +'/goal',str(current_goal))
#------------------------------------------------------------------------- 
		rate.sleep()
#-------------------------------------------------------------------------

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
 
 
 
 
