#!/bin/bash

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

# VNC password
VNC_PASSWORD=${PASSWORD:-ubuntu}

# Fix XDG_RUNTIME_DIR ownership
mkdir -p /tmp/runtime-ubuntu
chown $USER:$USER /tmp/runtime-ubuntu
chmod 700 /tmp/runtime-ubuntu

mkdir -p $HOME/.vnc
echo $VNC_PASSWORD | vncpasswd -f > $HOME/.vnc/passwd
chmod 600 $HOME/.vnc/passwd

touch $HOME/.Xauthority
chown $USER:$USER $HOME/.Xauthority

chown -R $USER:$USER $HOME
sed -i "s/password = WebUtil.getConfigVar('password');/password = '$VNC_PASSWORD'/" /usr/lib/novnc/app/ui.js

# xstartup
XSTARTUP_PATH=$HOME/.vnc/xstartup
cat << EOF > $XSTARTUP_PATH
#!/bin/sh
unset DBUS_SESSION_BUS_ADDRESS
mate-session
EOF
chown $USER:$USER $XSTARTUP_PATH
chmod 755 $XSTARTUP_PATH

# vncserver launch
VNCRUN_PATH=$HOME/.vnc/vnc_run.sh
cat << EOF > $VNCRUN_PATH
#!/bin/sh

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
user=ubuntu
command=/usr/bin/Xtigervnc :1 -geometry 1920x1080 -depth 24 -SecurityTypes None
environment=HOME="/home/ubuntu",USER="ubuntu",DISPLAY=":1"
#command=gosu '$USER' bash '$VNCRUN_PATH'
[program:novnc]
command=gosu '$USER' bash -c "websockify --web=/usr/lib/novnc 6080 localhost:5901"
EOF

# colcon
BASHRC_PATH=$HOME/.bashrc
grep -F "source /usr/share/gazebo/setup.sh" $BASHRC_PATH || echo "source /usr/share/gazebo/setup.sh" >> $BASHRC_PATH
grep -F "source /opt/ros/$ROS_DISTRO/setup.bash" $BASHRC_PATH || echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> $BASHRC_PATH
grep -F "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" $BASHRC_PATH || echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> $BASHRC_PATH
#
####################################################################
# This part is added by w.g.
####################################################################
#
echo '### Added by w.g.' >> $BASHRC_PATH
# A few aliases
echo '### Aliases' >> $BASHRC_PATH
echo 'alias rosdi="rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y"' >> $BASHRC_PATH
echo 'alias cbuild="colcon build --symlink-install"' >> $BASHRC_PATH
#

# After the existing aliases section, add:
echo '### Gazebo fixes' >> $BASHRC_PATH
echo 'export LIBGL_ALWAYS_SOFTWARE=1' >> $BASHRC_PATH
echo 'export MESA_GL_VERSION_OVERRIDE=3.3' >> $BASHRC_PATH
echo 'export XDG_RUNTIME_DIR=/tmp/runtime-ubuntu' >> $BASHRC_PATH
echo 'export GZ_SIM_RENDER_ENGINE=ogre' >> $BASHRC_PATH
echo 'alias gzsim="gz sim --render-engine ogre"' >> $BASHRC_PATH
####################################################################
#
chown $USER:$USER $BASHRC_PATH

# Fix rosdep permission
mkdir -p $HOME/.ros
cp -r /root/.ros/rosdep $HOME/.ros/rosdep
chown -R $USER:$USER $HOME/.ros

#EOF
chown -R $USER:$USER $HOME/Desktop

# clearup
PASSWORD=
VNC_PASSWORD=

echo "============================================================================================"
echo "NOTE: --security-opt seccomp=unconfined flag is required to launch Ubuntu Jammy based image."
echo -e 'See \e]8;;https://github.com/Tiryoh/docker-ros2-desktop-vnc/pull/56\e\\https://github.com/Tiryoh/docker-ros2-desktop-vnc/pull/56\e]8;;\e\\'
echo "============================================================================================"

exec /bin/tini -- supervisord -n -c /etc/supervisor/supervisord.conf