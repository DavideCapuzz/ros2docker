#!/bin/bash
# Robot Container Entrypoint
# Auto-configures VNC display based on VNC_PORT

set -e

# Auto-calculate DISPLAY from VNC_PORT (5900 + display_num = VNC_PORT)
# e.g., VNC_PORT=5901 -> DISPLAY=:1, VNC_PORT=5902 -> DISPLAY=:2
VNC_PORT=${VNC_PORT:-5901}
NOVNC_PORT=${NOVNC_PORT:-6080}
DISPLAY_NUM=$((VNC_PORT - 5900))
export DISPLAY=":${DISPLAY_NUM}"

echo "========================================="
echo "Auto-detected configuration:"
echo "  VNC Port: $VNC_PORT -> Display: $DISPLAY"
echo "  noVNC Port: $NOVNC_PORT"
echo "========================================="

# Configure User (user should already exist from Dockerfile)
USER=${USER:-ubuntu}
PASSWORD=${PASSWORD:-ubuntu}

if [ "$USER" = "root" ]; then
    HOME=/root
else
    HOME=/home/$USER

    # Verify user exists
    if ! id -u $USER > /dev/null 2>&1; then
        echo "ERROR: User $USER does not exist!"
        exit 1
    fi

    echo "* Configuring user: $USER"

    # Update password
    echo "$USER:$PASSWORD" | /usr/sbin/chpasswd 2> /dev/null || true

    # Ensure home directory ownership
    chown -R $USER:$USER ${HOME}
    [ -d "/dev/snd" ] && chgrp -R adm /dev/snd || true
fi

# VNC Setup
VNC_PASSWORD=${PASSWORD}

# Fix XDG_RUNTIME_DIR ownership
mkdir -p /tmp/runtime-$USER
chown $USER:$USER /tmp/runtime-$USER
chmod 700 /tmp/runtime-$USER

# Setup VNC password
mkdir -p $HOME/.vnc
echo "$VNC_PASSWORD" | vncpasswd -f > $HOME/.vnc/passwd
chmod 600 $HOME/.vnc/passwd
chown -R $USER:$USER $HOME/.vnc

# Setup Xauthority
touch $HOME/.Xauthority
chown $USER:$USER $HOME/.Xauthority

# Update noVNC password in UI
sed -i "s/password = WebUtil.getConfigVar('password');/password = '$VNC_PASSWORD'/" /usr/lib/novnc/app/ui.js

# Create xstartup script
XSTARTUP_PATH=$HOME/.vnc/xstartup
cat << 'EOF' > $XSTARTUP_PATH
#!/bin/sh
unset SESSION_MANAGER
unset DBUS_SESSION_BUS_ADDRESS
exec dbus-launch --exit-with-session openbox
EOF
chmod +x $XSTARTUP_PATH
chown $USER:$USER $XSTARTUP_PATH

# Create Supervisor configuration
mkdir -p /etc/supervisor/conf.d
CONF_PATH=/etc/supervisor/conf.d/supervisord.conf

cat << EOF > $CONF_PATH
[supervisord]
nodaemon=true
user=root
logfile=/var/log/supervisor/supervisord.log
logfile_maxbytes=50MB
logfile_backups=10
loglevel=info
pidfile=/var/run/supervisord.pid

[program:vnc]
user=${USER}
command=/usr/bin/Xtigervnc ${DISPLAY} -geometry 1920x1080 -depth 24 -SecurityTypes None -rfbport ${VNC_PORT}
environment=HOME="${HOME}",USER="${USER}",DISPLAY="${DISPLAY}",XDG_RUNTIME_DIR="/tmp/runtime-${USER}"
autorestart=true
autostart=true
priority=100
stdout_logfile=/var/log/supervisor/vnc.log
stderr_logfile=/var/log/supervisor/vnc_error.log
stdout_logfile_maxbytes=1MB
stderr_logfile_maxbytes=1MB

[program:novnc]
command=gosu '${USER}' bash -c "websockify --web=/usr/lib/novnc ${NOVNC_PORT} localhost:${VNC_PORT}"
environment=HOME="${HOME}",USER="${USER}"
autorestart=true
autostart=true
priority=200
stdout_logfile=/var/log/supervisor/novnc.log
stderr_logfile=/var/log/supervisor/novnc_error.log
stdout_logfile_maxbytes=1MB
stderr_logfile_maxbytes=1MB
EOF

# Setup bashrc with robot-specific configuration
BASHRC_PATH=$HOME/.bashrc

# Only append if not already present
if ! grep -q "ROS2 Multi-Robot Configuration" $BASHRC_PATH 2>/dev/null; then
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
fi

chown $USER:$USER $BASHRC_PATH

# Fix rosdep permissions
mkdir -p $HOME/.ros
if [ -d /root/.ros/rosdep ]; then
    cp -r /root/.ros/rosdep $HOME/.ros/ 2>/dev/null || true
fi
chown -R $USER:$USER $HOME/.ros

# Create Desktop directory
mkdir -p $HOME/Desktop
chown -R $USER:$USER $HOME/Desktop

# Create log directory
mkdir -p /var/log/supervisor
chown -R root:root /var/log/supervisor

echo "========================================="
echo "Container Configuration:"
echo "  User: $USER"
echo "  Display: $DISPLAY"
echo "  VNC Port: $VNC_PORT"
echo "  noVNC Port: $NOVNC_PORT"
echo "  Robot Name: ${ROBOT_NAME:-none}"
echo "  ROS Domain ID: ${ROS_DOMAIN_ID:-0}"
echo "========================================="

# Clear passwords from environment
unset PASSWORD
unset VNC_PASSWORD

# Execute the endfunction.sh if it exists, otherwise just keep supervisor running
if [ -f "/tmp/${ROBOT_NAME}/endfunction.sh" ]; then
    echo "Found endfunction.sh, will execute after supervisor starts..."

    # Start supervisor in background
    /bin/tini -- supervisord -c /etc/supervisor/supervisord.conf &
    SUPERVISOR_PID=$!

    # Wait for VNC to be ready
    echo "Waiting for VNC server to start..."
    for i in {1..30}; do
        if netstat -tlnp 2>/dev/null | grep -q ":${VNC_PORT}"; then
            echo "VNC server is ready!"
            break
        fi
        if [ $i -eq 30 ]; then
            echo "WARNING: VNC server did not start in time"
        fi
        sleep 1
    done

    # Execute the robot-specific function
    echo "Executing endfunction.sh..."
    gosu ${USER} bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && /tmp/${ROBOT_NAME}/endfunction.sh"

    # Keep supervisor running
    wait $SUPERVISOR_PID
else
    echo "No endfunction.sh found, starting supervisor only..."
    exec /bin/tini -- supervisord -n -c /etc/supervisor/supervisord.conf
fi
