#!/bin/bash

source /opt/ros/melodic/setup.sh
catkin_make -DBUILD_WITH_MARCH_NATIVE=ON -DUSE_PANGOLIN_VIEWER=OFF -DUSE_SOCKET_PUBLISHER=OFF -DUSE_STACK_TRACE_LOGGER=ON -DBOW_FRAMEWORK=DBoW2
source /home/docker_openvslam/catkin_ws/devel/setup.bash
rosrun openvslam run_slam -v src/openvslam/orb_vocab/orb_vocab.dbow2 -c src/openvslam/example/tum_rgbd/Husky.yaml --topic1 /zed_node/left/image_rect_gray --topic2 /zed_node/depth/depth_registered
