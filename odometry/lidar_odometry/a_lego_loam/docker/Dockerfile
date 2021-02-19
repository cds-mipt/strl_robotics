FROM osrf/ros:melodic-desktop-full

ARG USER=a_lego_loam
ARG UID=1000
ARG GID=1000
# default password
ARG PW=user

# Install dependencies
RUN apt-get update && apt-get install -q -y \
    mc \
    tmux \
    wget \
    libgoogle-glog-dev \
    # libatlas-base-dev \
    libsuitesparse-dev \
    libtbb-dev \
    ros-melodic-tf2-sensor-msgs \
    && rm -rf /var/lib/apt/lists/*

# Build and install Ceres
RUN wget ceres-solver.org/ceres-solver-1.14.0.tar.gz \
    && tar zxf ceres-solver-1.14.0.tar.gz \
    && mkdir ceres-bin \
    && cd ceres-bin \
    && cmake ../ceres-solver-1.14.0 \
    && make -j 7 \
    && make install

# Build and install gtsam
RUN wget -O gtsam-4.0.2.zip https://github.com/borglab/gtsam/archive/4.0.2.zip \
    && unzip gtsam-4.0.2.zip -d . \
    && cd gtsam-4.0.2 \
    && mkdir build && cd build \
    && cmake .. \
    && make -j 7 \
    && make install
ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib

# add user and his password
RUN useradd -m ${USER} --uid=${UID} && echo "${USER}:${PW}" | chpasswd && adduser ${USER} sudo
WORKDIR /home/${USER}
RUN mkdir -p catkin_ws/src && chown -R ${UID}:${GID} /home/${USER}
USER ${UID}:${GID}
