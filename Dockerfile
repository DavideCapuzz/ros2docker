# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# This Dockerfile is based on
# https://github.com/AtsushiSaito/docker-ubuntu-sweb
# and
# https://github.com/Tiryoh/docker-ros-desktop-vnc
# and
# https://github.com/wail-uottawa/docker-ros2-elg5228
# which are released under the Apache-2.0 license.

ARG ROS_VERSION=jazzy
FROM osrf/ros:${ROS_VERSION}-desktop-full AS stage-original
ENV ROS_DISTRO ${ROS_DISTRO}

LABEL maintainer "Davide Capuzzo"
MAINTAINER Davide Capuzzo

ARG TARGETPLATFORM

SHELL ["/bin/bash", "-c"]

######################################################

#    dos2unix               utility used to convert text files from DOS/Windows line endings (Carriage Return + Line Feed: CR+LF) to Unix line endings (
#    curl                   command-line tool for transferring data from or to a server using URLs
#    libglu1-mesa-dev
#    nano
#    evince
#    viewnior
#    filezilla
#    ruby-dev
#    tmux
#    wget
#    xorg-dev
#    zsh
#    iputils-ping

RUN apt-get update && apt-get install -y \
    dos2unix \
    curl \
    libglu1-mesa-dev \
    nano \
    evince \
    viewnior \
    filezilla \
    ruby-dev \
    tmux \
    wget \
    xorg-dev \
    zsh \
    iputils-ping

# Upgrade OS
RUN apt-get update -q && \
    DEBIAN_FRONTEND=noninteractive apt-get upgrade -y && \
    apt-get autoclean && \
    apt-get autoremove && \
    rm -rf /var/lib/apt/lists/*

########################################################################################
# if you need a fully desktop container uncomment the following lines
# Install Ubuntu Mate desktop
#RUN apt-get update -q && \
#    DEBIAN_FRONTEND=noninteractive apt-get install -y \
#        ubuntu-mate-desktop && \
#    apt-get autoclean && \
#    apt-get autoremove && \
#    rm -rf /var/lib/apt/lists/*
#
## Disable auto update and crash report
#RUN sed -i 's/Prompt=.*/Prompt=never/' /etc/update-manager/release-upgrades
#RUN sed -i 's/enabled=1/enabled=0/g' /etc/default/apport
########################################################################################
# minimal and light weight version of the tools for novnc grapichs
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
        x11-xserver-utils \
        xauth \
        xfonts-base \
        mesa-utils \
        libgl1 \
        libegl1 \
        openbox \
        libgl1-mesa-dri \
        libxkbcommon-x11-0 \
        libxcb-xinerama0 \
        libqt5gui5 \
        libqt5widgets5 \
        libqt5core5a \
        libqt5x11extras5 \
        dbus-x11 \
    && apt-get clean && rm -rf /var/lib/apt/lists/*
######################################################################################
ENV DISPLAY=:1
ENV QT_QPA_PLATFORM=xcb
ENV QT_X11_NO_MITSHM=1
ENV LIBGL_ALWAYS_SOFTWARE=1

#######
# for nvidia
# In your Dockerfile, after the existing ENV variables (around line 102), add:
ENV MESA_GL_VERSION_OVERRIDE=3.3
ENV MESA_GLSL_VERSION_OVERRIDE=330

# Ensure the XDG_RUNTIME_DIR is created with proper permissions
RUN mkdir -p /tmp/runtime-ubuntu && chmod 700 /tmp/runtime-ubuntu
#########################################################

# Add Package
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
        tigervnc-standalone-server tigervnc-common \
        supervisor wget curl gosu git python3-pip tini \
        build-essential vim sudo lsb-release locales \
        bash-completion tzdata terminator && \
    apt-get autoclean && \
    apt-get autoremove && \
    rm -rf /var/lib/apt/lists/*

RUN mkdir -p /home/ubuntu/.vnc && \
    printf '#!/bin/sh\nunset SESSION_MANAGER\nunset DBUS_SESSION_BUS_ADDRESS\ndbus-launch --exit-with-session openbox &\n' \
    > /home/ubuntu/.vnc/xstartup && \
    chmod +x /home/ubuntu/.vnc/xstartup && \
    chown -R ubuntu:ubuntu /home/ubuntu/.vnc
##################################################################################

# noVNC and Websockify
RUN git clone https://github.com/AtsushiSaito/noVNC.git -b add_clipboard_support /usr/lib/novnc
RUN pip install --break-system-packages git+https://github.com/novnc/websockify.git
RUN ln -s /usr/lib/novnc/vnc.html /usr/lib/novnc/index.html

# Set remote resize function enabled by default
RUN sed -i "s/UI.initSetting('resize', 'off');/UI.initSetting('resize', 'remote');/g" /usr/lib/novnc/app/ui.js

# A few tools
RUN apt-get update -q && \
    apt-get install -y \
    featherpad \
    doublecmd-qt

#####################################################################
# Add a few ROS packages
FROM stage-original AS stage-extra-ros2-packages

RUN apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-can-msgs \
    ros-$ROS_DISTRO-bondcpp \
    ros-$ROS_DISTRO-rosbag2-storage-mcap \
    nlohmann-json3-dev \
    libsuitesparse-dev \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-ros-gz-sim \
    ros-$ROS_DISTRO-ros-gz-bridge \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    python3-jinja2 \
    python3-typeguard \
    && rm -rf /var/lib/apt/lists/*

#####
# Install ceres solver for slam toolbox
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    libgoogle-glog-dev \
    libgflags-dev \
    libatlas-base-dev \
    libeigen3-dev \
    libsuitesparse-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt
RUN git clone https://github.com/ceres-solver/ceres-solver.git && \
    cd ceres-solver && \
    git checkout 2.2.0

RUN mkdir -p /opt/ceres-build && \
    cd /opt/ceres-build && \
    cmake ../ceres-solver \
      -DCMAKE_BUILD_TYPE=Release \
      -DBUILD_TESTING=OFF \
      -DBUILD_EXAMPLES=OFF \
      -DSUITESPARSE=ON \
      -DMINIGLOG=OFF \
      -DGFLAGS=ON && \
    make -j$(nproc) && \
    make install

WORKDIR /

#####################################################################
# Setup Workspace
# https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html
FROM stage-extra-ros2-packages AS stage-workspace

# ENV ROS_DISTRO humble
# desktop or ros-base
ARG INSTALL_PACKAGE=desktop

# ARG user=ubuntu
ENV USER=ubuntu
ENV HOME=/home/ubuntu
ENV ROS2_WS=$HOME/ros2_ws
#WORKDIR $HOME

RUN mkdir -p $ROS2_WS/src && \
    cd $ROS2_WS && \
    . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build --symlink-install

#####################################################################
# Finalization

FROM stage-workspace AS stage-finalization

RUN apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*

# Enable apt-get completion after running `apt-get update` in the container
RUN rm /etc/apt/apt.conf.d/docker-clean


COPY ./entrypoint.sh /
RUN chmod +x entrypoint.sh
RUN dos2unix /entrypoint.sh
ENTRYPOINT [ "/bin/bash", "-c", "/entrypoint.sh" ]

ENV USER ubuntu
ENV PASSWD ubuntu