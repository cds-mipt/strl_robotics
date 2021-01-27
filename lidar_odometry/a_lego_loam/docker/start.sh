#!/bin/bash
docker run -it --rm \
        --env="DISPLAY=$DISPLAY" \
        --env="QT_X11_NO_MITSHM=1" \
        --privileged \
        --name a_lego_loam \
        --net "host" \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        -v $1:/home/a_lego_loam/catkin_ws/src/a_lego_loam:rw \
        x64melodic/a_lego_loam:latest
