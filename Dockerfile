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

ARG ROS_DISTRO=jazzy
# Download the correct image form docker hub
FROM osrf/ros:${ROS_DISTRO}-desktop-full AS stage-base

LABEL maintainer="Davide Capuzzo"

ENV ROS_DISTRO=${ROS_DISTRO}
# Setup the the variable to run pakages without interactive way
ENV DEBIAN_FRONTEND=noninteractive

# setup the shell to use bash instead of sh
SHELL ["/bin/bash", "-c"]

# ============================================================================
# STAGE 1: Base System Setup
# ============================================================================

## Build tools
#    build-essential        metapackage that bundles essential tools like gcc, g++ (C/C++ compilers), and make
#    cmake                  pakage for build complex c++ packages
#    git                    version control software system
#    wget                   non-interactive command-line utility used to download files from the internet
#    curl                   command-line tool for transferring data from or to a server using URLs
#    python3-pip            package manager for Python 3, used to install and manage third-party software packages written in Python
## Utilities
#    dos2unix               utility used to convert text files from DOS/Windows line endings (Carriage Return + Line Feed: CR+LF) to Unix line endings (
#    nano                   command-line text editor
#    vim                    command-line text editor
#    tmux                   manage multiple terminal sessions within a single window
#    zsh                    advanced shell
#    iputils-ping           provides the essential ping command for network diagnostics
#    net-tools              provides the classic Linux networking commands (like ifconfig, netstat, route)
#    supervisor             used to monitor and manage a number of long-running processes
#    tini                   minimal init daemon specifically designed for use in containers
#    lsb-release            command-line tool that prints Linux Standard Base (LSB) and distribution-specific information
#    bash-completion        feature in the Bash shell that allows users to automatically complete commands
#    terminator             terminal emulator that lets you manage multiple terminal sessions
#    gosu             lightweight, simple substitute for sudo
## Graphics
#    libglu1-mesa-dev       header files and static libraries needed by programmers to compile applications that use OpenGL
#    mesa-utils             testing and diagnosing your system's open-source graphics stack
## Locales
#    locales                a set of language and regional settings that customize your system for specific cultural conventions, defining how things like dates, times, numbers, currency, and sorting are displayed and handled
#    tzdata                 collection of data defining the world's time zones
#    && rm -rf /var/lib/apt/lists/*     clean up evrything

RUN apt-get update && apt-get install -y \
## Build tools
    build-essential \
    cmake \
    git \
    wget \
    curl \
    python3-pip \
## Utilities
    dos2unix \
    nano \
    vim \
    tmux \
    zsh \
    iputils-ping \
    net-tools \
    supervisor \
    tini \
    lsb-release \
    bash-completion \
    terminator \
    gosu \
## Graphics
    libglu1-mesa-dev \
    mesa-utils \
## Locales
    locales \
    tzdata \
## Clean up
    && rm -rf /var/lib/apt/lists/*

# Upgrade OS
RUN apt-get update -q && \
    apt-get upgrade -y && \
    apt-get autoclean && \
    apt-get autoremove && \
    rm -rf /var/lib/apt/lists/*

# ============================================================================
# STAGE 2: Graphics & VNC Setup (for GUI containers)
# ============================================================================
FROM stage-base AS stage-graphics
########################################################################################
# if you need a fully desktop container uncomment the following lines
# Install Ubuntu Mate desktop
#RUN apt-get update -q && \
#    apt-get install -y \
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
# resources
# https://www.youtube.com/watch?v=mV1TNyWGQQ8
# https://en.wikipedia.org/wiki/X_Window_System

## Minimal desktop environment
#    x11-xserver-utils         Ubuntu is a package containing essential command-line tools for interacting with the X Window System (X11)
#    xauth                     command-line utility for managing .Xauthority files, which store cryptographic "cookies" (passwords) that authenticate X clients
#    xfonts-base               fundamental package providing a collection of low-resolution bitmapped fonts for the X Window System
#    openbox                   window manager that provides a minimal graphical interface
#    libgl1                    legacy compatibility library providing the old libGL.so.1 interface, essential for older applications needing OpenGL functions, acting as a wrapper that directs calls to the actual modern graphics drivers (like Mesa or NVIDIA's)
#    dbus-x11                  utility that bridges D-Bus (a message bus for app communication) with the X11 display system
#    libgl1                    library providing the OpenGL API (for 2D/3D graphics)
#    libgl1                    library providing the OpenGL API (for 2D/3D graphics)

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
## Minimal desktop environment
    x11-xserver-utils \
    xauth \
    xfonts-base \
    openbox \
    dbus-x11 \
## Graphics libraries
    libgl1 \
    libegl1 \
    libgl1-mesa-dri \
    libxkbcommon-x11-0 \
    libxcb-xinerama0 \
## Qt libraries
    libqt5gui5 \
    libqt5widgets5 \
    libqt5core5a \
    libqt5x11extras5 \
    && apt-get clean && rm -rf /var/lib/apt/lists/*
######################################################################################
# Forces Qt to use the X11 (xcb) backend.
# Qt supports multiple backends (Wayland, X11, offscreen). In containers, Qt sometimes guesses wrong and tries Wayland → crashes.
# xcb = stable X11 backend.
# Without it Qt apps may not start or crash immediately.
ENV QT_QPA_PLATFORM=xcb

# Disables MIT-SHM shared memory for Qt X11 rendering.
# Docker containers don’t share memory the same way as the host. MIT-SHM causes segmentation faults or freezes inside containers.
# Without it Random crashes or black windows.
ENV QT_X11_NO_MITSHM=1

# Forces software OpenGL rendering instead of GPU.
# Containers often don’t have access to host GPU drivers.
# Prevents OpenGL from trying (and failing) to use hardware acceleration.
ENV LIBGL_ALWAYS_SOFTWARE=1

#########################################################

## VNC server
#    tigervnc-standalone-server        Provides the VNC server (X server + VNC protocol)
#    tigervnc-common                   Shared libraries and utilities required by TigerVNC

RUN apt-get update && \
    apt-get install -y \
    tigervnc-standalone-server  \
    tigervnc-common && \
    apt-get autoclean && \
    apt-get autoremove && \
    rm -rf /var/lib/apt/lists/*

##################################################################################

# noVNC and Websockify
RUN git clone https://github.com/AtsushiSaito/noVNC.git -b add_clipboard_support /usr/lib/novnc
RUN pip install --break-system-packages git+https://github.com/novnc/websockify.git
# Makes http://host:port/ load noVNC automatically
RUN ln -s /usr/lib/novnc/vnc.html /usr/lib/novnc/index.html

# Set remote resize function enabled by default
RUN sed -i "s/UI.initSetting('resize', 'off');/UI.initSetting('resize', 'remote');/g" /usr/lib/novnc/app/ui.js

# ============================================================================
# STAGE 3: ROS2 Core Packages
# ============================================================================
# Add a few ROS packages
FROM stage-graphics AS stage-ros2-core

RUN apt-get update && apt-get install -y \
    ## Build and dev tools TBD
#    python3-colcon-common-extensions \
#    python3-colcon-clean \
#    python3-rosdep \
#    python3-vcstool \
#    ros-dev-tools \
    ## Common ROS2 packages
    ros-$ROS_DISTRO-can-msgs \
    ros-$ROS_DISTRO-bondcpp \
    ros-$ROS_DISTRO-rosbag2-storage-mcap \
    ## Communication middleware
#    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ## Development libraries
    nlohmann-json3-dev \
#    libsuitesparse-dev \
    python3-jinja2 \
    python3-typeguard \
    ## Gazebo bridge
#    ros-$ROS_DISTRO-ros-gz-sim \
#    ros-$ROS_DISTRO-ros-gz-bridge \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init || true && \
    rosdep update --rosdistro ${ROS_DISTRO}

# ============================================================================
# STAGE 4: Extra ROS2 Packages (with configurable profile)
# ============================================================================
FROM stage-ros2-core AS stage-extra-ros2-packages

ARG INSTALL_PROFILE=default
ARG ROBOT_NAME=robot1

COPY ./${ROBOT_NAME}/install_robot_deps.sh /tmp/install_robot_deps.sh

RUN chmod +x /tmp/install_robot_deps.sh && \
      /tmp/install_robot_deps.sh ${INSTALL_PROFILE}
# ============================================================================
# STAGE 5: Workspace Setup
# ============================================================================
# https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html
FROM stage-extra-ros2-packages AS stage-workspace

# ARG user=ubuntu
ENV USER=ubuntu
ENV HOME=/home/ubuntu
ENV ROS2_WS=$HOME/ros2_ws
#WORKDIR $HOME

RUN mkdir -p $ROS2_WS/src && \
    cd $ROS2_WS && \
    . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build --symlink-install

# ============================================================================
# STAGE 6: ROS2 + RViz Container (Finalization)
# ============================================================================
FROM stage-workspace AS stage-finalization
ARG ROBOT_NAME

RUN apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*

# Enable apt-get completion after running `apt-get update` in the container
RUN rm /etc/apt/apt.conf.d/docker-clean

ENV ROBOT_NAME=${ROBOT_NAME}

COPY ./entrypoint.sh /
RUN chmod +x entrypoint.sh
RUN dos2unix /entrypoint.sh

COPY ${ROBOT_NAME}/endfunction.sh /tmp/endfunction.sh
RUN chmod +x /tmp/endfunction.sh

ENTRYPOINT [ "/bin/bash", "-c", "/entrypoint.sh" ]

ENV USER ubuntu
ENV PASSWD ubuntu