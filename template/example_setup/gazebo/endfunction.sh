#!/bin/bash
set -e

# REQUIRED: source ROS
source /opt/ros/jazzy/setup.bash

# (optional) source overlay if it exists
if [ -f /home/ubuntu/ros2_ws/install/setup.bash ]; then
  source /home/ubuntu/ros2_ws/install/setup.bash
fi

cd /home/ubuntu/ros2_ws

echo "building model"
colcon build --packages-select multibot_model
echo "model built"

source install/setup.bash
echo "environment sourced"

ros2 launch multibot_model launch_world.py