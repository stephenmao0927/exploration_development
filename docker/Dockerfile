FROM ros:melodic-ros-base
# FROM ros:noetic-ros-base

ARG DEBIAN_FRONTEND=noninteractive
# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    rm /etc/localtime && \
    ln -s /usr/share/zoneinfo/Asia/Singapore /etc/localtime && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends tzdata

# Install basic apt tools
RUN apt update && apt upgrade -y && apt install -y \
    # Replace python-rosdep with python3-rosdep if running noetic
    vim wget zsh tmux

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV ROS_DISTRO melodic

# Env vars for the nvidia-container-runtime.
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute
# Create a non-root user and switch to it
RUN adduser --disabled-password --gecos '' --shell /bin/bash user 
# \&& chown -R user:user /app
RUN echo "user ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/90-user
USER user
# All users can use /home/user as their home directory
ENV HOME=/home/user
RUN chmod 777 /home/user

# Update to latest public mesa drivers for Ubuntu 18.04 LTS for wslg to hardware render - requires mesa v21 and above (automatic in Ubuntu 20.04 LTS and above)
# Can remove this code block if not planning to use wslg hardware render
RUN sudo apt install -y software-properties-common mesa-utils && \
    sudo add-apt-repository ppa:kisak/turtle && \
    sudo apt update && sudo apt dist-upgrade -y

# Install zsh related
RUN sh -c "$(wget -O- https://github.com/deluan/zsh-in-docker/releases/download/v1.1.2/zsh-in-docker.sh)" -- \
    -t ys \
    -p git \
	-p z \
    -p https://github.com/zsh-users/zsh-autosuggestions \
    -p https://github.com/zsh-users/zsh-syntax-highlighting \
    && sudo echo "set-option -g default-shell /bin/zsh" >> ~/.tmux.conf \
    && sudo chsh -s /bin/zsh

RUN sudo apt install -y python-scikits-learn \
    libopencv-dev \
    ros-${ROS_DISTRO}-rviz \
    ros-${ROS_DISTRO}-navigation


WORKDIR /home/user
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    mkdir -p /home/user/catkin_ws/src && \
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    echo "source /opt/ros/${ROS_DISTRO}/setup.zsh" >> ~/.zshrc

# Set up entrypoint
COPY hunter_exploration/docker/entrypoint.sh /entrypoint.sh
RUN sudo chmod +x /entrypoint.sh
# Set the default command to zsh
ENTRYPOINT [ "/entrypoint.sh" ] 
CMD zsh

