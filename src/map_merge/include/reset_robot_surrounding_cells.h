#ifndef RESET_ROBOT_SURROUNDING_CELLS_H
#define RESET_ROBOT_SURROUNDING_CELLS_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>

// the radius of robot is 0.15m, so the reset radius is 0.15m / 0.01m = 15 cells
const int reset_radius = 15 ;

bool worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my, const nav_msgs::OccupancyGrid& map);

bool withinGrid(int mx, int my, const nav_msgs::OccupancyGrid& map);

void resetRobotSurroundingCells(size_t robot_idx, int reset_radius, nav_msgs::OccupancyGrid& map);

#endif // RESET_ROBOT_SURROUNDING_CELLS_HPP
