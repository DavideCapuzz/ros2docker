#!/bin/bash
# install_robot_deps.sh
# Robot-specific dependencies installation script
# Usage: ./install_robot_deps.sh [profile]
# Profiles: minimal, navigation, perception, full

set -e

PROFILE=${1:-full}
ROS_DISTRO=${ROS_DISTRO:-jazzy}

echo "=========================================="
echo "Installing Robot Dependencies"
echo "Profile: ${PROFILE}"
echo "ROS Distro: ${ROS_DISTRO}"
echo "=========================================="

# ============================================================================
# CORE ROS2 PACKAGES (Always installed)
# ============================================================================
install_core_packages() {
    echo ">>> Installing Core ROS2 Packages..."
    apt-get update && apt-get install -y \
        python3-path \
        ros-${ROS_DISTRO}-rviz2 \
        ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
        && rm -rf /var/lib/apt/lists/*
}

# ============================================================================
# NAVIGATION & MAPPING
# ============================================================================
install_navigation_packages() {
    echo ">>> Installing Navigation Packages..."
    # Navigation2 stack
    # SLAM
    apt-get update && apt-get install -y \
        ros-${ROS_DISTRO}-navigation2 \
        ros-${ROS_DISTRO}-nav2-bringup \
        ros-${ROS_DISTRO}-nav2-map-server \
        ros-${ROS_DISTRO}-nav2-costmap-2d \
        ros-${ROS_DISTRO}-nav2-planner \
        ros-${ROS_DISTRO}-nav2-controller \
        ros-${ROS_DISTRO}-slam-toolbox \
        ros-${ROS_DISTRO}-cartographer \
        ros-${ROS_DISTRO}-cartographer-ros \
        && rm -rf /var/lib/apt/lists/*
}

# ============================================================================
# ROBOT CONTROL
# ============================================================================
install_control_packages() {
    echo ">>> Installing Control Packages..."
    # ros2_control
    # State and transforms
    # Localization
    apt-get update && apt-get install -y \
        ros-${ROS_DISTRO}-ros2-control \
        ros-${ROS_DISTRO}-ros2-controllers \
        ros-${ROS_DISTRO}-controller-manager \
        ros-${ROS_DISTRO}-control-msgs \
        ros-${ROS_DISTRO}-control-toolbox \
        ros-${ROS_DISTRO}-joint-state-publisher \
        ros-${ROS_DISTRO}-joint-state-publisher-gui \
        ros-${ROS_DISTRO}-robot-state-publisher \
        ros-${ROS_DISTRO}-robot-localization \
        && rm -rf /var/lib/apt/lists/*
}

# ============================================================================
# PERCEPTION (Optional - Heavy)
# ============================================================================
install_perception_packages() {
    echo ">>> Installing Perception Packages..."
    # Vision
    # Point clouds
    # Depth

    apt-get update && apt-get install -y \
        ros-${ROS_DISTRO}-vision-opencv \
        ros-${ROS_DISTRO}-cv-bridge \
        ros-${ROS_DISTRO}-image-transport \
        ros-${ROS_DISTRO}-compressed-image-transport \
        ros-${ROS_DISTRO}-perception-pcl \
        ros-${ROS_DISTRO}-pcl-conversions \
        ros-${ROS_DISTRO}-pcl-ros \
        ros-${ROS_DISTRO}-depth-image-proc \
        ros-${ROS_DISTRO}-image-pipeline \
        && rm -rf /var/lib/apt/lists/*
}

# ============================================================================
# CERES
# ============================================================================
install_ceres_libraries() {
    echo ">>> Installing Custom Libraries..."

    # Ceres Solver (for SLAM Toolbox)
    apt-get update && apt-get install -y \
        build-essential \
        cmake \
        git \
        libgoogle-glog-dev \
        libgflags-dev \
        libatlas-base-dev \
        libeigen3-dev \
        libsuitesparse-dev \
        && rm -rf /var/lib/apt/lists/*

    echo "Building Ceres Solver..."
    cd /opt
    if [ ! -d "ceres-solver" ]; then
        git clone https://github.com/ceres-solver/ceres-solver.git
        cd ceres-solver
        git checkout 2.2.0
        mkdir -p build && cd build
        cmake .. \
            -DCMAKE_BUILD_TYPE=Release \
            -DBUILD_TESTING=OFF \
            -DBUILD_EXAMPLES=OFF \
            -DSUITESPARSE=ON \
            -DMINIGLOG=OFF \
            -DGFLAGS=ON
        make -j$(nproc)
        make install
        cd /opt && rm -rf ceres-solver
    else
        echo "Ceres already installed, skipping..."
    fi
}

# ============================================================================
# Gazebo
# ============================================================================
install_gazebo_bridge_libraries() {
    echo ">>> Installing Custom Libraries..."
    ## Gazebo
    apt-get update && apt-get install -y \
        ros-${ROS_DISTRO}-ros-gz-bridge \
        ros-${ROS_DISTRO}-ros-gz-sim \
        ros-${ROS_DISTRO}-xacro \
        && rm -rf /var/lib/apt/lists/*
}

# ============================================================================
# CUSTOM LIBRARIES (Ceres, etc.)
# ============================================================================
install_custom_libraries() {
    echo ">>> Installing Custom Libraries..."
    apt-get update && apt-get install -y \
    && rm -rf /var/lib/apt/lists/*
}

# ============================================================================
# DEFAULT
# ============================================================================
install_default_libraries() {
    echo "not installing anything"
}

# ============================================================================
# PROFILE SELECTION
# ============================================================================
case "${PROFILE}" in
    minimal)
        install_simulation_packages
        ;;

    navigation)
        install_simulation_packages
        install_navigation_packages
        install_control_packages
        install_custom_libraries
        ;;

    perception)
        install_simulation_packages
        install_navigation_packages
        install_control_packages
        install_perception_packages
        install_custom_libraries
        ;;

    full)
        install_simulation_packages
        install_navigation_packages
        install_control_packages
        install_perception_packages
        install_custom_libraries
        ;;

    custom)
        install_core_packages
        install_custom_libraries
        install_gazebo_bridge_libraries
        ;;

    gazebo)
        install_gazebo_bridge_libraries
        ;;

    default)
        install_default_libraries
        ;;

    *)
        echo "Unknown profile: ${PROFILE}"
        echo "Available profiles: minimal, navigation, perception, full"
        exit 1
        ;;
esac

echo "=========================================="
echo "✓ Installation Complete!"
echo "=========================================="
