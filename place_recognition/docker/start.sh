#!/bin/bash

if [ $# != 1 ]
then
    echo "Usage:
          bash start.sh [DATABASE_DIR]
         "
    exit 1
fi

get_real_path(){
  if [ "${1:0:1}" == "/" ]; then
    echo "$1"
  else
    realpath -m "$PWD"/"$1"
  fi
}

DATABASE_DIR=$(get_real_path "$1")
echo "Reference database directory: "$DATABASE_DIR

if [ ! -d $DATABASE_DIR ]
then
    echo "ERROR: DATABASE_DIR=$DATABASE_DIR is not an existing directory."
exit 1
fi

orange=`tput setaf 3`
reset_color=`tput sgr0`

export ARCH=`uname -m`

cd "$(dirname "$0")"
root_dir=$PWD 
cd $root_dir

echo "Running on ${orange}${ARCH}${reset_color}"

if [ "$ARCH" == "x86_64" ] 
then
    ARGS="--ipc host --gpus all -e NVIDIA_DRIVER_CAPABILITIES=all"
elif [ "$ARCH" == "aarch64" ] 
then
    ARGS="--runtime nvidia"
else
    echo "Arch ${ARCH} not supported"
    exit
fi

xhost +
    docker run -it -d --rm \
        $ARGS \
        --env="DISPLAY=$DISPLAY" \
        --env="QT_X11_NO_MITSHM=1" \
        --privileged \
        --name strl_place_recognition \
        --net host \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        -v `pwd`/../place_recognition/:/home/docker_place_recognition/catkin_ws/src/place_recognition:rw \
        -v $DATABASE_DIR/:/home/docker_place_recognition/reference_db:rw \
        ${ARCH}noetic/strl_place_recognition:latest
xhost -