#!/bin/bash

cd $(dirname $0)

NUM_THREADS=${1:-$(nproc)}

docker build . \
    --build-arg UID=$(id -g) \
    --build-arg GID=$(id -g) \
    --build-arg NUM_THREADS=${NUM_THREADS} \
    -f Dockerfile \
    -t rtabmap:latest
