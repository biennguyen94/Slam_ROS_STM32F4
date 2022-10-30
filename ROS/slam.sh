#!/bin/bash

source ~/catkin_ws/devel/setup.bash
export ROS_PACKAGE_PATH=~/catkin_ws/devel/:$ROS_PACKAGE_PATH

roslaunch `rospack find patrick_the_robot`/launch/slam.launch
#read -p "Press [Enter] key to end ..."
