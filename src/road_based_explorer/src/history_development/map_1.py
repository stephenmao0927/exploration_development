#!/usr/bin/env python3
#coding: utf-8 

import rospy
from nav_msgs.msg import OccupancyGrid

def occupancy_grid_publisher():
    rospy.init_node('occupancy_grid_publisher', anonymous=True)
    pub = rospy.Publisher('map', OccupancyGrid, queue_size=10)
    rate = rospy.Rate(10)  # 发布频率为10Hz

    # 创建OccupancyGrid消息对象
    occupancy_grid = OccupancyGrid()
    occupancy_grid.header.stamp = rospy.Time.now()
    occupancy_grid.header.frame_id = 'map'
    occupancy_grid.info.resolution = 0.1  # 栅格分辨率
    occupancy_grid.info.width = 1200  # 栅格宽度
    occupancy_grid.info.height = 750  # 栅格高度
    occupancy_grid.info.origin.position.x = 0.0  # 栅格原点x坐标
    occupancy_grid.info.origin.position.y = 0.0  # 栅格原点y坐标

    # 设置栅格数据
    occupancy_data = [-1] * (occupancy_grid.info.width * occupancy_grid.info.height)
    
    # 设置马路可通行区域
    road_width = 30
    road_start = occupancy_grid.info.height // 2 - road_width // 2
    road_end = road_start + road_width
    for y in range(road_start, road_end):
        for x in range(occupancy_grid.info.width):
            index = y * occupancy_grid.info.width + x
            occupancy_data[index] = 0  # 可通行
    occupancy_grid.data = occupancy_data

    while not rospy.is_shutdown():
        # 发布occupancy grid消息
        pub.publish(occupancy_grid)
        rate.sleep()

if __name__ == '__main__':
    try:
        occupancy_grid_publisher()
    except rospy.ROSInterruptException:
        pass
