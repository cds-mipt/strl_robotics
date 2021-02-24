#!/bin/bash

source /opt/ros/melodic/setup.sh
catkin_make -j4
source /home/a_lego_loam/catkin_ws/devel/setup.bash
roslaunch alego husky_mipt.launch
