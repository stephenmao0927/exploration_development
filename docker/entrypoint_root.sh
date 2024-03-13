#!/bin/zsh
. ~/.zshrc
cd /root/catkin_ws
catkin_make
echo "source /root/catkin_ws/devel/setup.zsh" >> ~/.zshrc
cd 
exec "$@"