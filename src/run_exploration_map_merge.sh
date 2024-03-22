#!/bin/zsh
# This script must run in docker.
# Created and maintained by Stephen Mao (maok0002@e.ntu.edu.sg)

source /opt/ros/melodic/setup.zsh
source /home/user/catkin_ws/devel/setup.zsh

# rostopic pub /robot_1/ackermann_steering_controller/cmd_vel geometry_msgs/Twist "linear:
#   x: 1.0
#   y: 0.0
#   z: 0.0
# angular:
#   x: 0.0
#   y: 0.0
#   z: 0.0" -r 10 &
# ros_pub_pid1=$!

# rostopic pub /robot_2/ackermann_steering_controller/cmd_vel geometry_msgs/Twist "linear:
#   x: 1.0
#   y: 0.0
#   z: 0.0
# angular:
#   x: 0.0
#   y: 0.0
#   z: 0.0" -r 10 &
# ros_pub_pid2=$!

# sleep 6

# kill $ros_pub_pid1
# kill $ros_pub_pid2

# roslaunch road_based_explorer multi_robots_map_merging.launch &
roslaunch road_based_explorer multiple_test_traversability.launch
# roslaunch road_based_explorer single_test_road_based.launch