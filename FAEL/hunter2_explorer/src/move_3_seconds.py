#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import time

# Initialize node
rospy.init_node('multi_robot_cmd_vel_publisher')

# Define Publishers for each robot
pub1 = rospy.Publisher('/robot_1/ackermann_steering_controller/cmd_vel', Twist, queue_size=10)
pub2 = rospy.Publisher('/robot_2/cmd_vel', Twist, queue_size=10)
pub3 = rospy.Publisher('/robot_3/cmd_vel', Twist, queue_size=10)

# Define the cmd_vel message
twist = Twist()

# Set linear velocity
twist.linear.x = 1.0

# Sleep time to ensure that the node is fully initialized
time.sleep(2)

# Publish cmd_vel message to each robot
pub1.publish(twist)
pub2.publish(twist)
pub3.publish(twist)

# Wait 3 seconds
rospy.sleep(5)

# Set linear velocity to zero
twist.linear.x = 0.0

# Publish cmd_vel message to each robot
pub1.publish(twist)
pub2.publish(twist)
pub3.publish(twist)

# Sleep time to ensure that the node is fully initialized
