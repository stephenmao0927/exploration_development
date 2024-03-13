#!/bin/zsh
. ~/.zshrc
cd /home/user/catkin_ws
catkin_make
echo "source /home/user/catkin_ws/devel/setup.zsh" >> ~/.zshrc
cd 
exec "$@"