#!/bin/bash

orange=`tput setaf 3`
reset_color=`tput sgr0`

ARCH=`uname -m`

if [ $ARCH != "x86_64" ]
then
    echo "Architecture ${orange}${ARCH}${reset_color} is not supported"
    exit 1
fi

cd "$(dirname "$0")"
root_dir=$PWD 
cd $root_dir/..

echo "Building for ${orange}${ARCH}${reset_color}"

docker build . \
    -f docker/Dockerfile.${ARCH} \
    --build-arg UID=$(id -u) \
    --build-arg GID=$(id -g) \
    -t ${ARCH}noetic/strl_place_recognition:latest