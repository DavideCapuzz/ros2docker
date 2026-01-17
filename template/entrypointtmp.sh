#!/bin/bash
# endfunction.sh - Robot startup script
# This runs after VNC is ready, inside supervisor

set -e

ROBOT_NAME=${ROBOT_NAME:-unknown}

echo "=========================================="
echo "Starting robot: ${ROBOT_NAME}"
echo "ROS_DISTRO: ${ROS_DISTRO}"
echo "ROS_DOMAIN_ID: ${ROS_DOMAIN_ID}"
echo "DISPLAY: ${DISPLAY}"
echo "=========================================="

# IMPROVED: Wait for ROS environment to be ready
if [ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
    echo "ERROR: ROS distribution ${ROS_DISTRO} not found!"
    exit 1
fi

# Source ROS2
source /opt/ros/${ROS_DISTRO}/setup.bash

# IMPROVED: Source workspace if it exists and is built
if [ -f "${HOME}/ros2_ws/install/setup.bash" ]; then
    echo "✓ Sourcing workspace..."
    source ${HOME}/ros2_ws/install/setup.bash
else
    echo "⚠ Workspace not built yet, skipping..."
fi

# IMPROVED: Wait for display to be available
echo "Waiting for display ${DISPLAY} to be ready..."
for i in {1..10}; do
    if xdpyinfo -display ${DISPLAY} >/dev/null 2>&1; then
        echo "✓ Display is ready"
        break
    fi
    sleep 1
done

# IMPROVED: Check ROS2 daemon
echo "Checking ROS2 daemon..."
ros2 daemon status || ros2 daemon start

# IMPROVED: Print ROS2 environment info
echo "ROS2 Environment:"
echo "  RMW_IMPLEMENTATION: ${RMW_IMPLEMENTATION:-default}"
echo "  ROS_LOCALHOST_ONLY: ${ROS_LOCALHOST_ONLY:-0}"
echo "  ROS_DOMAIN_ID: ${ROS_DOMAIN_ID}"

# =================================================================
# ROBOT-SPECIFIC STARTUP CODE GOES HERE
# =================================================================

# Example: Launch a specific robot application
# ros2 launch my_robot bringup.launch.py robot_name:=${ROBOT_NAME}

# Example: Start a specific node
# ros2 run my_package my_node --ros-args -r __ns:=/${ROBOT_NAME}

# IMPROVED: For now, just keep container alive and log status
echo "=========================================="
echo "✓ Robot ${ROBOT_NAME} startup complete"
echo "Container will stay alive. Use 'docker exec' to interact."
echo "=========================================="

# Keep the script running (supervisor will manage this)
# Use 'tail -f /dev/null' instead of 'sleep infinity' for better signal handling
exec tail -f /dev/null
