#!/bin/bash

python3 change_frame.py --sub_topic /OpenVSLAM/odom --pub_topic /visual_odom_in_lidar_frame --calib_filename '/home/administrator/Repos/robot_localization/calib/cam2lidar.yml' --nodename visual2lidar --frame_id lidar_odom --child_frame_id lidar &
python3 add_covariance.py --cov_filename '/home/administrator/Repos/robot_localization/calib/openvslam_covariance.txt' --nodename add_cov_visual --sub_topic /visual_odom_in_lidar_frame --pub_topic /visual_odom &
python3 add_covariance.py --cov_filename '/home/administrator/Repos/robot_localization/calib/a_lego_loam_covariance.txt' --nodename add_cov_lidar --sub_topic /odom_aft_mapped --pub_topic /lidar_odom --frame_id lidar_odom --child_frame_id lidar
