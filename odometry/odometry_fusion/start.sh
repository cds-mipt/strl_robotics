#!/bin/bash

source /opt/ros/melodic/setup.sh
catkin_make -j4
source /home/odometry_fusion/catkin_ws/devel/setup.bash
roslaunch robot_localization husky_mipt_odometry_fusion.launch
