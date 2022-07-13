#!/bin/bash
docker exec --user "docker_place_recognition" -it strl_place_recognition \
        /bin/bash -c "source /opt/ros/noetic/setup.bash; cd /home/docker_place_recognition; echo STRL Place Recognition ROS-Noetic container; echo ; /bin/bash"
