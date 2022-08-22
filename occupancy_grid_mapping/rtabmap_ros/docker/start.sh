#!/bin/bash

docker_dir=$(dirname $0)

docker run -it -d --rm \
    --ipc host \
    --gpus all \
    --env="NVIDIA_DRIVER_CAPABILITIES=all" \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --privileged \
    --net "host" \
    --name rtabmap \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v $(realpath $docker_dir)/../../../:/home/docker_rtabmap/catkin_ws:rw \
    rtabmap:latest
