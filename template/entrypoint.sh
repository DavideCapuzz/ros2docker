#!/bin/bash
# Robot Container Entrypoint - Single Variable Configuration

# ============================================
# CONFIGURATION FROM ENVIRONMENT
# ============================================
DISPLAY=${DISPLAY:-:1}
DISPLAY_NUM=${DISPLAY:1}

# Use explicit ports from environment, or calculate from DISPLAY
VNC_PORT=${VNC_PORT:-$((5900 + DISPLAY_NUM))}
NOVNC_PORT=${NOVNC_PORT:-$((6080 + DISPLAY_NUM - 1))}

echo "========================================="
echo "Configuration:"
echo "  DISPLAY: ${DISPLAY}"
echo "  VNC Port: ${VNC_PORT}"
echo "  noVNC Port: ${NOVNC_PORT}"
echo "  Robot: ${ROBOT_NAME:-default}"
echo "========================================="

# Create User
USER=${USER:-root}
HOME=/root
if [ "$USER" != "root" ]; then
    echo "* enable custom user: $USER"
    useradd --create-home --shell /bin/bash --user-group --groups adm,sudo $USER 2>/dev/null || echo "User already exists"
    echo "$USER ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
    if [ -z "$PASSWORD" ]; then
        echo "  set default password to \"ubuntu\""
        PASSWORD=ubuntu
    fi
    HOME=/home/$USER
    echo "$USER:$PASSWORD" | /usr/sbin/chpasswd 2> /dev/null || echo ""
    cp -r /root/{.config,.gtkrc-2.0,.asoundrc} ${HOME} 2>/dev/null || true
    chown -R $USER:$USER ${HOME}
    [ -d "/dev/snd" ] && chgrp -R adm /dev/snd
fi

# VNC Setup
VNC_PASSWORD=${PASSWORD}

# Fix XDG_RUNTIME_DIR ownership
mkdir -p /tmp/runtime-$USER
chown $USER:$USER /tmp/runtime-$USER
chmod 700 /tmp/runtime-$USER

# Fix /tmp permissions
chmod 1777 /tmp

# Setup VNC password
mkdir -p $HOME/.vnc
echo "$VNC_PASSWORD" | vncpasswd -f > $HOME/.vnc/passwd
chmod 600 $HOME/.vnc/passwd
chown -R $USER:$USER $HOME/.vnc

# Setup Xauthority
touch $HOME/.Xauthority
chown $USER:$USER $HOME/.Xauthority

# Update noVNC password in UI
#sed -i "s/password = WebUtil.getConfigVar('password');/password = '$VNC_PASSWORD'/" /usr/lib/novnc/app/ui.js
if [ -f /usr/share/novnc/app/ui.js ]; then
    sed -i "s/password = WebUtil.getConfigVar('password');/password = '$VNC_PASSWORD'/" /usr/share/novnc/app/ui.js
elif [ -f /usr/lib/novnc/app/ui.js ]; then
    sed -i "s/password = WebUtil.getConfigVar('password');/password = '$VNC_PASSWORD'/" /usr/lib/novnc/app/ui.js
fi
# Create xstartup script
XSTARTUP_PATH=$HOME/.vnc/xstartup
cat << EOF > $XSTARTUP_PATH
#!/bin/sh
unset DBUS_SESSION_BUS_ADDRESS
EOF
chown $USER:$USER $XSTARTUP_PATH
chmod 755 $XSTARTUP_PATH
#EOF

# Supervisor
# Create Supervisor configuration
mkdir -p /etc/supervisor/conf.d
CONF_PATH=/etc/supervisor/conf.d/supervisord.conf

cat << EOF > $CONF_PATH
[supervisord]
nodaemon=true
user=root

[program:vnc]
user=${USER}
command=/usr/bin/Xtigervnc ${DISPLAY} -geometry 1920x1080 -depth 24 -localhost -SecurityTypes None
environment=HOME="${HOME}",USER="${USER}",DISPLAY="${DISPLAY}"
autorestart=true
priority=100

[program:novnc]
user=${USER}
command=/usr/bin/websockify --web=/usr/share/novnc ${NOVNC_PORT} localhost:${VNC_PORT}
autorestart=true
priority=200

[program:robot_startup]
user=${USER}
command=/bin/bash /tmp/endfunction.sh
directory=/home/${USER}
environment=HOME="/home/${USER}",USER="${USER}",ROS_DISTRO="jazzy",ROBOT_NAME="${ROBOT_NAME}",DISPLAY="${DISPLAY}"
autorestart=false
startsecs=0
priority=300
stdout_logfile=/var/log/supervisor/robot_startup.log
stderr_logfile=/var/log/supervisor/robot_startup_error.log
EOF

# Setup bashrc with robot-specific configuration
BASHRC_PATH=$HOME/.bashrc
cat >> $BASHRC_PATH << 'BASHRC_EOF'
# ========================================
# ROS2 Multi-Robot Configuration
# ========================================

# Source ROS2
if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then
    source /opt/ros/${ROS_DISTRO}/setup.bash
fi

# Source workspace (if built)
if [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
fi

# Colcon autocomplete
if [ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash ]; then
    source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
fi

# Robot-specific configuration
export ROBOT_NAME=${ROBOT_NAME}
export ROBOT_NAMESPACE=${ROBOT_NAMESPACE:-}
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}

# Use simulation time if configured
if [ "${USE_SIM_TIME}" == "true" ]; then
    export ROS_USE_SIM_TIME=1
fi

# Graphics fixes
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_GL_VERSION_OVERRIDE=3.3
export XDG_RUNTIME_DIR=/tmp/runtime-${USER}
export GZ_SIM_RENDER_ENGINE=ogre

BASHRC_EOF

chown $USER:$USER $BASHRC_PATH

# Fix rosdep permissions
mkdir -p $HOME/.ros
if [ -d /root/.ros/rosdep ]; then
    cp -r /root/.ros/rosdep $HOME/.ros/rosdep 2>/dev/null || true
fi
chown -R $USER:$USER $HOME/.ros 2>/dev/null || true

# Cleanup
PASSWORD=
VNC_PASSWORD=

# Start supervisor in background
/bin/tini -- supervisord -c /etc/supervisor/supervisord.conf &

# Wait for VNC to be ready
echo "Waiting for VNC server on ${DISPLAY} to be ready..."
for i in {1..30}; do
    if DISPLAY=${DISPLAY} xdpyinfo >/dev/null 2>&1; then
        echo "✓ VNC server is ready on ${DISPLAY}"
        break
    fi
    if [ $i -eq 30 ]; then
        echo "✗ VNC server failed to start after 30 seconds"
        exit 1
    fi
    sleep 1
done

# Wait for supervisor (keeps container running)
wait
