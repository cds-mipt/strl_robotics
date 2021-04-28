#!/bin/bash

#файл должен находится в Repos/solov2_pytorch1.7/bash_scripts/ на роботе
./docker/latest/start_devel.sh
cd ../solov2_utils/
docker cp camera_objects_msgs solo:/home/docker_solo/catkin_ws/src/
docker cp camera_objects_visualizer solo:/home/docker_solo/catkin_ws/src/

docker exec -it --user docker_solo \
            solo bash -c "cd /home/docker_solo/; 
			source /opt/ros/melodic/setup.bash;
			cd catkin_ws;
			catkin_make;
			source devel/setup.bash;
			cd src/solov2/solo_ros/launch/;
			echo 'For start write -> roslaunch zed_depth_test.launch';
			bash"
