# BASIC SETUP

# Official pytorch container
FROM nvcr.io/nvidia/cuda:11.1-cudnn8-devel-ubuntu18.04

ARG USER=docker_solo
ARG UID=1000
ARG GID=1000
# default password
ARG PW=user


# GENERAL SETUP

# Expose default SSH port
EXPOSE 22

# Install Miniconda3
ENV CONDADIR /opt/miniconda3
ENV CONDAEXE ${CONDADIR}/bin/conda
RUN apt-get update && apt-get install -y wget \
    && wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh 2>/dev/null \
    && bash Miniconda3-latest-Linux-x86_64.sh -b -p ${CONDADIR} \
    && useradd -m anaconda --uid=6666 && echo "anaconda:anaconda" | chpasswd \
    && chown -R anaconda:anaconda ${CONDADIR} \
    && chmod -R g+w ${CONDADIR} \
    && ${CONDAEXE} clean -a -y \
    && rm -f Miniconda3-latest-Linux-x86_64.sh

# Adding keys for ROS
RUN apt-get update && apt-get install -y curl && rm -rf /var/lib/apt/lists/*
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros-latest.list' 
RUN curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | apt-key add -

# Installing ROS
RUN apt-get update && DEBIAN_FRONTEND="noninteractive" apt-get -y install tzdata && rm -rf /var/lib/apt/lists/*
RUN apt-get update \
    && apt-get install -y ros-melodic-desktop-full \
    python-rosdep \
    python-rosinstall \
    python-rosinstall-generator \
    python-wstool \
    build-essential \
    && rm -rf /var/lib/apt/lists/*
RUN rosdep init && rosdep update

# Install system dependencies for convinient development inside container
RUN apt-get update && apt-get install -y \
    ros-melodic-desktop-full \
    tmux \
    openssh-server \
    tree \
    less \
    vim \
    curl \
    wget \
    build-essential \
    && rm -rf /var/lib/apt/lists/*


# COMPONENT SPECIFIC SETUP

# Add user and his password
RUN useradd -m ${USER} --uid=${UID} \
    && echo "${USER}:${PW}" | chpasswd \
    && usermod -s /bin/bash ${USER}

WORKDIR /home/${USER}

# For ability install deps from container shell during environment setup
RUN usermod -a -G sudo ${USER} \
    && usermod -a -G anaconda ${USER}

# Create some dirs
RUN mkdir -p catkin_ws/src && chown -R ${UID}:${GID} ./

# Configure conda for ${USER}
RUN su -c "${CONDAEXE} init" ${USER} \
    && su -c "${CONDAEXE} config --set auto_activate_base false" ${USER}


# METHOD SPECIFIC SETUP
# ADD YOUR COMMANDS START FROM HERE

# All needed for model inference, i.e. weights, configs etc. NOT FOR DATASETS!
VOLUME [ "/home/${USER}/model" ]

# Install additional system dependencies here
RUN apt-get update \
    && apt-get install -y \
       libglib2.0-0 \
       libsm6 \
       libxrender-dev \
       libxext6 \
       python-pip \
    && rm -rf /var/lib/apt/lists/*

# Install system python dependencies
RUN python -m pip --no-cache-dir install git+https://github.com/ivbelkin/io_shared.git@rc-0.0.4
RUN python -m pip --no-cache-dir install "git+https://github.com/cocodataset/cocoapi.git#subdirectory=PythonAPI"

# Create conda environment
USER ${USER}
RUN ${CONDAEXE} create -n solo python=3.7 -y
RUN ${CONDAEXE} config --add channels https://mirrors.tuna.tsinghua.edu.cn/anaconda/cloud/pytorch/
RUN ${CONDAEXE} config --set show_channel_urls yes
RUN ${CONDAEXE} install -n solo pytorch torchvision cudatoolkit=11.0
COPY requirements/build.txt requirements.txt
RUN ${CONDAEXE} run -n solo python -m pip --no-cache-dir install -r requirements.txt
RUN ${CONDAEXE} run -n solo python -m pip --no-cache-dir install "git+https://github.com/cocodataset/cocoapi.git#subdirectory=PythonAPI"
RUN ${CONDAEXE} run -n solo python -m pip --no-cache-dir install git+https://github.com/ivbelkin/io_shared.git@rc-0.0.4
USER root

# Finally copy the code and install. Do it always at the end of Dockerfile
COPY --chown=${USER}:${USER} . /home/${USER}/catkin_ws/src/solov2
