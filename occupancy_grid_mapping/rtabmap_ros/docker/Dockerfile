FROM osrf/ros:noetic-desktop-full

ENV DEBIAN_FRONTEND noninteractive

ARG NUM_THREADS=1

# User setup
ARG USER=docker_rtabmap
ARG UID=1000
ARG GID=1000
ARG PW=user

# Add user and his password
RUN useradd -m ${USER} --uid=${UID} && \
    echo "${USER}:${PW}" | chpasswd && \
    usermod -s /bin/bash ${USER} && \
    usermod -a -G sudo ${USER}

# Install dependencies
RUN apt-get update && apt-get install -y \
    git \
    ros-noetic-rtabmap ros-noetic-rtabmap-ros && \
    apt-get remove -y ros-noetic-rtabmap ros-noetic-rtabmap-ros && \
    rm -rf /var/lib/apt/lists/*

RUN mkdir -p /home/${USER}/catkin_ws/src && \
    chown -R ${UID}:${GID} /home/${USER}

WORKDIR /home/${USER}
USER ${UID}:${GID}
