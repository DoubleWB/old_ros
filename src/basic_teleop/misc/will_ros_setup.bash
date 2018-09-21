#!/bin/bash 
cd ~/catkin_ws
catkin_make
cd devel
. setup.bash
export ROS_MASTER_URI=http://192.168.0.101:11311/
export ROS_IP=192.168.0.101
cd ..
cd ..
