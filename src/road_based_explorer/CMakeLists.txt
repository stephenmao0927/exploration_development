cmake_minimum_required(VERSION 3.0.2)
project(road_based_explorer)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
  geometry_msgs
  rrt_exploration
  )

add_message_files(
  FILES
  RoboPointArray.msg
  RoboArcArray.msg
)
 
add_service_files(
   FILES
   transfer_goal.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  rrt_exploration
 )

catkin_package( 
CATKIN_DEPENDS rospy std_msgs message_runtime)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

