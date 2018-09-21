#!/bin/bash 
sudo chown root.gpio /dev/gpiomem
sudo chmod g+rw /dev/gpiomem
cd ~/tobot_ws/catkin_ws2
catkin_make
cd devel
. setup.bash
export ROS_MASTER_URI=http://10.0.0.52:11311/
export ROS_IP=10.0.0.49
cd ~

