
#include <occupancy_grid_utils/coordinate_conversions.h>
#include <reset_robot_surrounding_cells.h>

bool worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my, const nav_msgs::OccupancyGrid& map) {
  mx = (unsigned int)((wx - map.info.origin.position.x) / map.info.resolution);
  my = (unsigned int)((wy - map.info.origin.position.y) / map.info.resolution);

  // Return false if out of map bounds
  if (mx < 0 || my < 0 || mx >= map.info.width || my >= map.info.height) {
    return false;
  }

  return true;
}

bool withinGrid(int mx, int my, const nav_msgs::OccupancyGrid& map) {
  if (mx < 0 || my < 0 || mx >= map.info.width || my >= map.info.height) {
    return false;
  }

  return true;
}

// update the surrounding cells of the robot in merged_map_
void resetRobotSurroundingCells(size_t robot_idx, int reset_radius, nav_msgs::OccupancyGrid& map) {
  // Transform Listener
  tf::TransformListener listener_;
  tf::StampedTransform transform_;

  std::string target_frame = "/robot_" + std::to_string(robot_idx + 1) + "/base_footprint";
  ROS_INFO("Start resetting surrounding cells! Target frame: %s", target_frame.c_str());

  try {
    // wait for the transform to be available
    listener_.waitForTransform("/map_merged", target_frame, ros::Time(0), ros::Duration(1.0));
    // get the transform
    listener_.lookupTransform("/map_merged", target_frame, ros::Time(0), transform_);
    // output the transform and the target_frame for debug
    ROS_INFO("Transform Found: %f %f %f", transform_.getOrigin().x(), transform_.getOrigin().y(), transform_.getOrigin().z());
  }
  catch (tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
    return;
  }

  // convert transform to grid coordinates
  unsigned int mx, my;
  // get the map coordinates of the robot from transform_
  // the reset radius is in map coordinates，so we need to convert the robot coordinates to map coordinates
  // the resolution of the map is 0.01m, so we need to iterate the cells in the reset radius
  if (worldToMap(transform_.getOrigin().x(), transform_.getOrigin().y(), mx, my, map)) {
    // reset the surrounding cells
    for (int dx = -reset_radius; dx <= reset_radius; dx++) {
      for (int dy = -reset_radius; dy <= reset_radius; dy++) {
        if (withinGrid(mx + dx, my + dy, map)) {
          const occupancy_grid_utils::Cell cell(mx + dx, my + dy);
          const occupancy_grid_utils::index_t ind = cellIndex(map.info, cell);
          map.data[ind] = 0;
        }
      }
    }
  }
}
