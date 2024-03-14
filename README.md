## Compile
### 1. Build on host
**Ubuntu18.04 + ROS Melodic**
### 2. Build using docker
#### 2.1 Pulling my image from Dockerhub
```bash
# if you build container on wsl2
docker-compose -f docker-compose.yml up
# for deploying on hunter
docker-compose -f docker-compose-exploration-deploy.yml
```
#### 2.2 Building an image manually
**Notice:** I built the docker on wsl2

- Build Image
```bash
# docker
cd hunter_exploration/docker
# build image 
. build.sh
```
- Mount container
```bash
# if you build container on wsl2
# modify the name of your image
docker-compose -f docker-compose.yml up
# if you build container on linux system
# modify the name of your image
docker-compose -f docker-compose_server2 up
```
#### 2.3 Launch simulation world in docker
```bash
# get in
docker exec -it hunter_exploration zsh
roslaunch hunter2_explorer three_robots.launch
```
### dependencies of new detector
```bash
pip install transforms3d==0.3.1 rospkg
```
## Update
### **2023.08.07 update:** test two robots demo

1. How to run two robots demo
```bash
# in simulation docker
roslaunch hunter2_explorer multiple_simulated_city_teb.launch
# in traversability_analysis docker
roslaunch traversability_mapping run_mapping_fastlio_two_robots.launch
# in exploration docker
# launch map merge node
roslaunch new_detector multi_robots_map_merging.launch
# launch exploration node
roslaunch new_detector multiple_test_traversability.launch
```
2. generate exploation area based on the `top_left` and `bottom_right` points
```yaml
<include file="$(find new_detector)/launch/include/start_filter.launch">
    <arg name="robot_name" value="robot_2"/>
    <arg name="exploation_points" value="[(x1, y1), (x2, y2)]"/>
</include>
```
`(x1,y1)` is the top left point, `(x2,y2)` is the bottom right point

### 2024.03.14 update: using a single script to start exploration in simulation

```bash
# in simulation docker
roslaunch hunter2_explorer multiple_simulated_city_teb.launch
# in traversability_analysis docker
roslaunch traversability_mapping run_mapping_fastlio_two_robots.launch
# in exploration docker
sudo zsh run_exploration_map_merge.sh
```

This script will handle all the preparation work, including publishing cmd to drive the robot  for a few seconds to make all the nearby area known, and start the exploration and map merge algorithm.

## Debug log

### 1. The "multirobot_map_merge" has been updated, some users have reported problems getting correct map merging. However, the old version still works fine.
**Solution:** Git clone [old version](https://github.com/hasauino/m-explore) instead of ```sudo apt install ros-melodic-multirobot-map-merge ros-melodic-explore-lite```

### 2. The distances between robots are not correct, i.e. 3 meters in gazebo but 6 meters in rviz.
No error when subscribing map from traversability analysis
### 3. three robots are considered obstacles on merged map
add `reset_robot_surrounding_cells.cpp` to reset cells to `0`