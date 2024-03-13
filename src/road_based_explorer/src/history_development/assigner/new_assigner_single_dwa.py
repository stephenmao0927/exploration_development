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
from functions import robot,informationGain,discount
from numpy.linalg import norm
from transforms3d.euler import quat2euler

current_goal = None

class Turn():
    def __init__(self, index, direction):
        self.index = index
        self.direction = direction
        self.cmd_vel = rospy.Publisher('robot_'+ str(self.index) +'/cmd_vel', Twist, queue_size=10)
        
        r = rospy.Rate(10)
        move_cmd = Twist()
        if direction == "left":
            move_cmd.linear.x = 0.0
            move_cmd.angular.z = 1.0
        
        if direction == "right":
            move_cmd.linear.x = 0.0
            move_cmd.angular.z = -1.0

        self.cmd_vel.publish(move_cmd)
        r.sleep()

class Stop():
    def __init__(self, index):
        self.index = index
        self.cmd_vel = rospy.Publisher('robot_'+ str(self.index) +'/cmd_vel', Twist, queue_size=10)
        
        r = rospy.Rate(10)
        move_cmd = Twist()
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 0

        self.cmd_vel.publish(move_cmd)
        r.sleep()

def yaw_proceed(orientation):
    _, _, yaw = quat2euler(
            [orientation.w,
            orientation.x,
            orientation.y,
            orientation.z])
    return yaw  

# Subscribers' callbacks------------------------------
mapData=OccupancyGrid()
frontiers=[]
global1=OccupancyGrid()
global2=OccupancyGrid()
global3=OccupancyGrid()
globalmaps=[]

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
	map_topic= rospy.get_param('~map_topic','/robot_1/move_base_node/global_costmap/costmap')
	info_radius= rospy.get_param('~info_radius',1.0)					#this can be smaller than the laser scanner range, >> smaller >>less computation time>> too small is not good, info gain won't be accurate
	info_multiplier=rospy.get_param('~info_multiplier',3.0)		
	hysteresis_radius=rospy.get_param('~hysteresis_radius',3.0)			#at least as much as the laser scanner range
	hysteresis_gain=rospy.get_param('~hysteresis_gain',2.0)				#bigger than 1 (biase robot to continue exploring current region
	frontiers_topic= rospy.get_param('~frontiers_topic','/filtered_points')	
	n_robots = rospy.get_param('~n_robots',1)
	namespace = rospy.get_param('~namespace','')
	namespace_init_count = rospy.get_param('namespace_init_count',1)
	delay_after_assignement=rospy.get_param('~delay_after_assignement',0.5)
	rateHz = rospy.get_param('~rate',100)
	dbg = rospy.get_param('~debug', True)
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
	print(n_robots)
	if len(namespace)>0:
		for i in range(0,n_robots):
			robots.append(robot(namespace+str(i+namespace_init_count)))
			# if dbg:
			# 	rospy.logdebug("robot initialization:", namespace+str(i+namespace_init_count))
	elif len(namespace)==0:
		a = robot('')
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
#Get information gain for each frontier point
		infoGain=[]
		for ip in range(0,len(centroids)):
			infoGain.append(informationGain(mapData,[centroids[ip][0],centroids[ip][1]],info_radius))
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
					vel = rospy.wait_for_message('/robot_1/cmd_vel',Twist)
					
					if vel.linear.x < 0.1:
						if dis > 4:
							odom = rospy.wait_for_message('/robot_1/Odometry',Odometry)
							yaw_robot = yaw_proceed(odom.pose.pose.orientation)
							cur_position = [odom.pose.pose.position.x,odom.pose.pose.position.y]
							yaw_goal = math.atan2((current_goal[1] - cur_position[1]),(current_goal[0] - cur_position[0]))
							dyaw = yaw_robot - yaw_goal
							while abs(dyaw) > 0.2:
								if dyaw > 0:
									Turn(1,'right')
								elif dyaw < 0:
									Turn(1,'left')
								
								odom = rospy.wait_for_message('/robot_1/Odometry',Odometry)
								yaw_robot = yaw_proceed(odom.pose.pose.orientation)
								yaw_goal = math.atan2((current_goal[1] - cur_position[1]),(current_goal[0] - cur_position[0]))
								dyaw = yaw_robot - yaw_goal

							Stop(1)
						
						else:
							waiting_count += 1
							print(waiting_count)

					if dis < 1.5 :
						rospy.set_param('~/robot_'+str(namespace_init_count+ir)+'/get_goal_status','ready')

					# if dis < 1.5 or waiting_count > 25:
					if dis < 1.2 or waiting_count > 10:
						for ip in range(0,len(centroids)):
							cost=norm(robots[ir].getPosition()-centroids[ip])	
							revenue = 999 - cost
							revenue_record.append(revenue)
							centroid_record.append(centroids[ip])
							id_record.append(ir)
					else:
						break
						
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
			rospy.loginfo(max(revenue_record))
			rospy.loginfo(revenue_record)
			robots[id_record[winner_id]].sendGoal(centroid_record[winner_id])
			current_goal = centroid_record[winner_id]
			waiting_count = 0
			current_robot = str(namespace_init_count+id_record[winner_id])
			# rospy.set_param('~/robot_'+current_robot+'/new_path_ready','ready')
			rospy.loginfo(namespace+str(namespace_init_count+id_record[winner_id])+"  assigned to  "+str(centroid_record[winner_id]))	
			rospy.sleep(delay_after_assignement)
		
		if current_goal is not None:
			rospy.set_param('~/robot_'+current_robot+'/goal',str(current_goal))
#------------------------------------------------------------------------- 
		rate.sleep()
#-------------------------------------------------------------------------

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
 
 
 
 
