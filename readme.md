# FAEL: Built Using Docker

### 1. Compose docker

1.1 Download my docker image from docker hub.

```bash
docker pull stephenmao0927/fael:v2
```

1.2 Use docker compose to compile the docker.

```bash
cd ~/fael_docker_source/docker
docker-compose -f docker-compose-fael.yml up
```

Remember to modify `source` tag in `docker-compose-fael.yml` according to where the source folder locates on your host.

To enter docker:

```bash
docker exec -it fael_test zsh
```

### 2. Download gazebo models

```zsh
cd .gazebo
git clone https://github.com/osrf/gazebo_models.git
mv gazebo_models models
```

The last step is necessary as `~/.gazebo/models`  is the default path for gazebo to load all the models.

### 3. Compile and Run the demo

```bash
cd
cd catkin_ws
catkin_make
source devel/setup.zsh
```

To run the demo:

```bash
# To launch the env
roslaunch exploration_manager sim_env.launch
# To launch the local planner
roslaunch exploration_manager robot_move.launch
# To start the exploration
roslaunch exploration_manager explorer.launch
```

