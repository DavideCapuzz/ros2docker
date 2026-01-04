#!/bin/bash
# Robot Container Entrypoint
# Supports: Multi-robot namespacing, CLion remote dev, Gazebo spawning
# Create User
USER=${USER:-root}
HOME=/root
if [ "$USER" != "root" ]; then
    echo "* enable custom user: $USER"
    useradd --create-home --shell /bin/bash --user-group --groups adm,sudo $USER
    echo "$USER ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
    if [ -z "$PASSWORD" ]; then
        echo "  set default password to \"ubuntu\""
        PASSWORD=ubuntu
    fi
    HOME=/home/$USER
    echo "$USER:$PASSWORD" | /usr/sbin/chpasswd 2> /dev/null || echo ""
    cp -r /root/{.config,.gtkrc-2.0,.asoundrc} ${HOME} 2>/dev/null
    chown -R $USER:$USER ${HOME}
    [ -d "/dev/snd" ] && chgrp -R adm /dev/snd
fi

# VNC Setup
VNC_PASSWORD=${PASSWORD}
# Fix XDG_RUNTIME_DIR ownership
mkdir -p /tmp/runtime-$USER
chown $USER:$USER /tmp/runtime-$USER
chmod 700 /tmp/runtime-$USER

mkdir -p $HOME/.vnc
echo $VNC_PASSWORD | vncpasswd -f > $HOME/.vnc/passwd
chmod 600 $HOME/.vnc/passwd

touch $HOME/.Xauthority
chown $USER:$USER $HOME/.Xauthority

# Update noVNC password
sed -i "s/password = WebUtil.getConfigVar('password');/password = '$VNC_PASSWORD'/" /usr/lib/novnc/app/ui.js

# xstartup
XSTARTUP_PATH=$HOME/.vnc/xstartup
cat << EOF > $XSTARTUP_PATH
#!/bin/sh
unset DBUS_SESSION_BUS_ADDRESS
EOF
chown $USER:$USER $XSTARTUP_PATH
chmod 755 $XSTARTUP_PATH

if [ $(uname -m) = "aarch64" ]; then
    LD_PRELOAD=/lib/aarch64-linux-gnu/libgcc_s.so.1 vncserver :1 -fg -geometry 1920x1080 -depth 24
else
    vncserver :1 -fg -geometry 1920x1080 -depth 24
fi
EOF

# Supervisor
CONF_PATH=/etc/supervisor/conf.d/supervisord.conf
cat << EOF > $CONF_PATH
[supervisord]
nodaemon=true
user=root

[program:vnc]
user=${USER}
command=/usr/bin/Xtigervnc :1 -geometry 1920x1080 -depth 24 -SecurityTypes None
environment=HOME="${HOME}",USER="${USER}",DISPLAY=":1"
autorestart=true
priority=100

[program:novnc]
command=gosu '${USER}' bash -c "websockify --web=/usr/lib/novnc ${NOVNC_PORT} localhost:${VNC_PORT}"
autorestart=true
priority=200
EOF

# Setup bashrc with robot-specific configuration
BASHRC_PATH=$HOME/.bashrc
cat >> $BASHRC_PATH << 'BASHRC_EOF'
# ========================================
# ROS2 Multi-Robot Configuration
# ========================================

# Source ROS2
source /opt/ros/${ROS_DISTRO}/setup.bash

# Source workspace (if built)
if [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
fi

# Colcon autocomplete
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash

# Robot-specific namespace
export ROBOT_NAME=${ROBOT_NAME}
export ROBOT_NAMESPACE=${ROBOT_NAMESPACE}
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID}

# Use simulation time if configured
if [ "${USE_SIM_TIME}" == "true" ]; then
    export ROS_USE_SIM_TIME=1
fi

# Graphics fixes
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_GL_VERSION_OVERRIDE=3.3
export XDG_RUNTIME_DIR=/tmp/runtime-${USER}

# @TODO check if necessary
export GZ_SIM_RENDER_ENGINE=ogre

BASHRC_EOF

chown $USER:$USER $BASHRC_PATH

# Fix rosdep permissions
mkdir -p $HOME/.ros
if [ -d /root/.ros/rosdep ]; then
    cp -r /root/.ros/rosdep $HOME/.ros/rosdep
fi
chown -R $USER:$USER $HOME/.ros

#EOF
chown -R $USER:$USER $HOME/Desktop

# clearup
PASSWORD=
VNC_PASSWORD=

# Start services
exec /bin/tini -- supervisord -n -c /etc/supervisor/supervisord.conf