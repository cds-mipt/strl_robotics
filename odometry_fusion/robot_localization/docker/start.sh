#!/bin/bash
docker run -it --rm \
        --env="DISPLAY=$DISPLAY" \
        --env="QT_X11_NO_MITSHM=1" \
        --privileged \
        --name odometry_fusion \
        --net "host" \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        -v $1:/home/odometry_fusion/catkin_ws/src/robot_localization:rw \
        x64melodic/odometry_fusion:latest
