#!/bin/zsh
. ~/.zshrc
cd /root/catkin_ws
# catkin_make -DCATKIN_BLACKLIST_PACKAGES="rs_to_velodyne"
# echo "export LD_LIBRARY_PATH=$MMDEPLOY_DIR/build/lib:$TENSORRT_DIR/lib:$CUDNN_DIR/lib:$LD_LIBRARY_PATH" >> ~/.zshrc
# echo "export PYTHONPATH=$MMDEPLOY_DIR/build/lib:$PYTHONPATH" >> ~/.zshrc
echo "source /root/catkin_ws/devel/setup.zsh" >> ~/.zshrc
cd 
exec "$@"