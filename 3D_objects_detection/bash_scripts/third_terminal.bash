#!/bin/bash

#файл должен находится в Repos/solov2_pytorch1.7/bash_scripts/ на роботе
docker exec -it --user docker_solo \
            solo bash -c "cd /home/docker_solo/; 
			source /opt/ros/melodic/setup.bash;
			cd catkin_ws;
			source devel/setup.bash;
			cd src/camera_objects_visualizer/launch;
			echo 'For start write -> roslaunch zed_depth_test_visual.launch';
			bash"



