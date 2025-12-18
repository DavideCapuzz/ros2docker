# Set the ROS distribution as an argument, defaulting to 'jazzy'
ARG ROS_DISTRO=jazzy
ARG USER_PC=docker_ros

# The base image comes from the official ROS repository hosted on Docker Hub
# You can find available ROS images here: https://hub.docker.com/_/ros/tags
ARG BASE_IMAGE=osrf/ros:jazzy-desktop-full
FROM ${BASE_IMAGE}

# Set the default shell to bash for RUN commands
# This ensures all RUN commands use bash instead of sh
SHELL ["/bin/bash", "-c"]

# Update the system and install essential tools
# This step upgrades all packages and installs utilities needed for development

# curl A command-line tool for getting or sending data using URL syntax https://curl.se/
# nano text editor https://www.nano-editor.org/
# evince document viewer https://wiki.gnome.org/Apps/Evince
# viewnior image viewr https://siyanpanayotov.com/project/viewnior/
# filezilla ftp transfer file https://filezilla-project.org/
# ruby-dev ruby compiler https://www.ruby-lang.org/en/
# tmux terminal multiplexer https://github.com/tmux/tmux/wiki
# wget non-interactive command-line utility for downloading files from the internet https://www.gnu.org/software/wget/
# xorg-dev impelmentation of xwindows system https://packages.debian.org/sid/xorg-dev
# zsh customizable command-line shell for docker_ros https://github.com/ohmyzsh/ohmyzsh/wiki/Installing-ZSH
# iputils-ping network diagnostic tool https://packages.debian.org/sid/iputils-ping
# bzip2 used fro zip file
# file file management
# git distributed version control software system https://git-scm.com/
# vim text editor
# dos2unix for converting line endings
# VNC and noVNC related packages
RUN apt-get update && apt-get install -y \
    curl \
    nano \
    evince \
    viewnior \
    filezilla \
    ruby-dev \
    tmux \
    wget \
    xorg-dev \
    zsh \
    iputils-ping \
    bzip2 \
    file \
    git \
    vim \
    dos2unix \
    sudo && \
    # Clean up apt lists to reduce image size
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y  \
    fluxbox \
    xfce4 \
    xvfb \
    x11vnc \
    supervisor \
    net-tools \
    novnc \
    websockify && \
     # Clean up apt lists to reduce image size
     rm -rf /var/lib/apt/lists/*

# Install VNC and noVNC packages in separate layer
#RUN apt-get update && apt-get install -y \
#    tigervnc-standalone-server \
#    tigervnc-common \
#    fluxbox \
#    xterm \
#    net-tools \
#    dbus-x11 \
#    libglx-mesa0 \
#    libgl1-mesa-dri \
#    x11-apps && \
#    # Clean up apt lists to reduce image size
#    rm -rf /var/lib/apt/lists/*

# Install noVNC
#RUN git clone https://github.com/novnc/noVNC.git /opt/noVNC && \
#    git clone https://github.com/novnc/websockify /opt/noVNC/utils/websockify && \
#    ln -s /opt/noVNC/vnc.html /opt/noVNC/index.html

#RUN apt-get update && apt-get install -y  \
#    git python3 py3-pip
RUN rm -rf /usr/share/novnc
RUN git clone https://github.com/novnc/noVNC.git /usr/share/novnc
RUN git clone https://github.com/novnc/websockify.git /usr/share/novnc/utils/websockify
RUN ln -sf /usr/share/novnc/vnc.html /usr/share/novnc/index.html

#####################################################################
# Setup Workspace
# https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html

# desktop or ros-base
ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID

ENV USER=${USERNAME}
ENV HOME=/home/${USERNAME}
ENV ROS2_WS=${HOME}/ros2_ws

# Create user with specified UID/GID (or modify existing user)
RUN if id -u $USERNAME > /dev/null 2>&1; then \
      echo "User $USERNAME already exists, modifying..."; \
      usermod -u $USER_UID -d /home/$USERNAME -s /bin/bash $USERNAME; \
    else \
      groupadd --gid $USER_GID $USERNAME && \
      useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME; \
    fi && \
    mkdir -p /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config && \
    echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME
#RUN groupadd --gid ${USER_GID} ${USERNAME} && \
#    useradd -m -s /bin/bash --uid ${USER_UID} --gid ${USER_GID} ${USERNAME} && \
#    echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} && \
#    chmod 0440 /etc/sudoers.d/${USERNAME}
# Switch to the new user
USER $USERNAME
WORKDIR $HOME

# Create and build ROS2 workspace
RUN mkdir -p ${ROS2_WS}/src && \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    cd ${ROS2_WS} && colcon build --symlink-install

# Add ROS setup to bashrc
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc && \
    echo "source $ROS2_WS/install/setup.bash" >> ~/.bashrc

#####################################################################
# Finalization

# Switch back to root for cleanup and entrypoint setup
USER root

# Copy and setup entrypoint script
#COPY entrypoint.sh /entrypoint.sh
RUN #chmod +x /entrypoint.sh && dos2unix /entrypoint.sh

# Expose ports for noVNC and VNC
#EXPOSE 8080 5900 6080

# Create VNC password directory
#RUN mkdir -p /home/$USERNAME/.vnc && chown -R $USERNAME:$USERNAME /home/$USERNAME/.vnc

ENV DISPLAY=:1
ENV RESOLUTION=1920x1080x24

EXPOSE 6080 5900
# Set vnc password
#ARG VNC_PASS=dummypass
#
## Create vnc password file
#RUN mkdir -p /root/.vnc && \
#    x11vnc -storepasswd "$VNC_PASS" /root/.vnc/passwd
ARG VNC_PASS=ros
ENV VNC_PASS=${VNC_PASS}

RUN mkdir -p /home/${USERNAME}/.vnc && \
    x11vnc -storepasswd "${VNC_PASS}" /home/${USERNAME}/.vnc/passwd && \
    chown -R ${USERNAME}:${USERNAME} /home/${USERNAME}/.vnc
COPY supervisord.conf /etc/supervisord.conf
USER $USERNAME
CMD ["supervisord", "-c", "/etc/supervisord.conf", "-n"]

# Switch back to user for runtime
#USER $USERNAME

#ENTRYPOINT ["/entrypoint.sh"]