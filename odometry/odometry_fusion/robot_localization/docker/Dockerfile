FROM osrf/ros:melodic-desktop-full

ARG USER=odometry_fusion
ARG UID=1000
ARG GID=1000
# default password
ARG PW=user

# Install dependencies
RUN apt-get update && apt-get install -q -y \
    mc \
    tmux \
    wget \
    libgeographic-dev \
    ros-melodic-geographic-msgs \
    libyaml-cpp-dev \
    && rm -rf /var/lib/apt/lists/*

# add user and his password
RUN useradd -m ${USER} --uid=${UID} && echo "${USER}:${PW}" | chpasswd && adduser ${USER} sudo
WORKDIR /home/${USER}
RUN mkdir -p catkin_ws/src && chown -R ${UID}:${GID} /home/${USER}
USER ${UID}:${GID}
