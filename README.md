# ROS2Docker

ROS2 and Docker are two powerful tools for modern programming that generate strong feelings—both love and hate—from many programmers. To increase the love for these tools, I created **ros2docker**: a modular, general-purpose template for creating multiple ROS2 Docker containers simultaneously for development with high customization and flexibility while maintaining an easy-to-understand structure.

This flexible template allows you to create multiple different containers with different images and packages installed inside. Every robot is unique, which is why ros2docker provides multiple customization options that let you install your packages independently from the robot configuration without modifying the ros2docker repository itself.

You can find additional information on [my blog](https://davidecapuzz.github.io/docker/2025/12/06/Robot-Development-with-Docker.html).

> **Note:** The setup is currently tested using Linux with CLion/VSCode.

---

## Quick Start

1. Clone this repo into your `.devcontainer` folder
2. Start the devcontainer using `devcontainer.json`
3. Enjoy your custom ROS2 environment!

---

## How It Works

When you start the container from `.devcontainer.json`, a bash script is automatically called. This script configures the environment according to your customizations found in the `.devcontainer` folder.

---

## Customization Options

To achieve a highly customized environment, you can create your own configuration in your `.devcontainer` folder. This approach allows you to use the same setup across multiple environments without modifying the git repository.

For modular setup, you can customize:
- `docker-compose.yml`
- `entrypoint.sh`
- `endfunction.sh`
- Package dependencies in `install_robot_deps.sh`

> **Important:** You don't need to copy and modify all files. Any files not defined in your `.devcontainer` folder will be taken from the template.

---

### docker-compose.yml

There are **two ways** to customize the Docker image:

#### Method 1: Direct docker-compose.yml
Define the `docker-compose.yml` file directly in your `.devcontainer` folder. You can find examples in the `template` folder.

#### Method 2: INI Configuration File (Recommended)
Use a `setup.ini` file to specify parameters for each container. All parameters are optional:

**Available Parameters:**
- `install_profile` - Installation package profile used in `install_robot_deps.sh` (default: `default`)
- `ros_install` - Base ROS image (default: `osrf/ros:${ROS_DISTRO}-desktop-full`)
  - Example: `ros:jazzy-ros-base-noble`
- `ros_domain` - ROS domain ID (default: `0`)
- `network` - Docker network name (default: empty)
- `volumes` - Persistent volumes (default: empty)
  - Example: `build,install,log`
- `endfunction` - Path to custom endfunction script (default: template)
- `env` - Custom environment variables (default: empty)
  - Example: `STARTPOS_X:7;STARTPOS_Y:2;STARTPOS_Z:0`

**Example setup.ini:**
```ini
[gazebo]
install_profile=gazebo
ros_install=ros:jazzy-ros-base-noble
ros_domain=0
network=ros2_network
volumes=
endfunction=gazebo/endfunction.sh

[robot1]
install_profile=custom
ros_install=ros:jazzy-ros-base-noble
ros_domain=0
network=ros2_network
volumes=build,install,log
env=STARTPOS_X:7;STARTPOS_Y:2;STARTPOS_Z:0
```

With either method, the startup script will generate the `docker-compose.yml` file in the `tmp` folder inside the ros2docker directory.

---

### entrypoint.sh

The entrypoint file is responsible for:
- Setting up `supervisor.conf` and `.bashrc` files
- Starting all applications defined in `supervisor.conf`

**Customization:**
If you want to modify the entrypoint, create your custom `entrypoint.sh` file in the `.devcontainer` folder. The script will automatically copy it to the `tmp` folder before building the container image.

---

### endfunction.sh

The endfunction file is responsible for running custom commands such as:
- `ros2 launch` commands
- `colcon build` operations
- Other robot-specific startup tasks

**Customization:**
To create a custom endfunction:
1. Create a folder with your robot's name inside the `.devcontainer` folder
2. Place your `endfunction.sh` file inside that folder
3. Reference it in your `setup.ini` with the `endfunction` parameter

The script will automatically copy the file to the `tmp` folder before copying it to the container image.

**Example:**
```
.devcontainer/
├── setup.ini
├── robot1/
│   └── endfunction.sh
├── gazebo/
│   └── endfunction.sh
├── ros2docker/
    └── ....
```

---

### install_robot_deps.sh

he `install_robot_deps.sh` script manages package installation based on predefined profiles. It supports modular installation of ROS2 packages, dependencies, and custom libraries.

#### Available Profiles

| Profile | Description | Packages Installed |
|---------|-------------|-------------------|
| `default` | No packages | Skips installation entirely |
| `minimal` | Basic simulation | Simulation packages only |
| `navigation` | Navigation stack | Simulation + Navigation2 + SLAM + Control |
| `perception` | Computer vision | Navigation + Vision + PCL + Depth processing |
| `full` | Complete stack | All packages including perception |
| `custom` | Custom setup | Core ROS2 + Custom libraries + Gazebo bridge |
| `gazebo` | Gazebo only | Gazebo bridge and simulation tools |

#### Customization

To add custom packages or create your own installation profiles:

1. Copy `install_robot_deps.sh` to your `.devcontainer` folder
2. Add your custom installation functions
3. Add a new case in the profile selection section
4. Reference your profile in `setup.ini`

**Example: Adding a Custom Profile**
```bash
# Add your custom installation function
install_my_custom_packages() {
    echo ">>> Installing My Custom Packages..."
    apt-get update && apt-get install -y \
        ros-${ROS_DISTRO}-my-package \
        python3-my-dependency \
        && rm -rf /var/lib/apt/lists/*
}

# Add to profile selection
case "${PROFILE}" in
    # ... existing profiles ...
    
    my_custom)
        install_core_packages
        install_my_custom_packages
        ;;
    
    *)
        echo "Unknown profile: ${PROFILE}"
        exit 1
        ;;
esac
```

**Then use it in setup.ini:**
```ini
[my_robot]
install_profile=my_custom
ros_domain=0
```

#### Package Categories

The default script organizes packages into categories:

- **Core Packages**: Essential ROS2 tools (rviz2, python3-path)
- **Navigation**: Navigation2 stack, SLAM Toolbox, Cartographer
- **Control**: ros2_control, controllers, state publishers, robot localization
- **Perception**: Vision (OpenCV, cv_bridge), Point clouds (PCL), Depth processing
- **Custom Libraries**: Ceres Solver for advanced SLAM
- **Gazebo**: Gazebo bridge, simulation tools, XACRO

---

## Project Structure

```
ros2docker/
├── devcontainer.json
├── Dockerfile
├── scripts
│   ├── copy_endfunction.sh
│   ├── init_docker-compose.sh
│   └── init_setup.sh
└── template
    ├── docker-compose.yml
    ├── endfunction.sh
    ├── entrypoint.sh
    ├── example_setup
    │   ├── gazebo
    │   │   └── endfunction.sh
    │   └── setup.ini
    └── install_robot_deps.sh

```

---

## Usage Example

**Minimal setup.ini:**
```ini
[my_robot]
install_profile=custom
ros_domain=5
volumes=build,install,log
```

**With custom environment variables:**
```ini
[my_robot]
install_profile=custom
ros_domain=5
volumes=build,install,log
env=ROBOT_ID:1;MAX_SPEED:2.5;DEBUG:true
```

---

## Contributing

Contributions are welcome! Please feel free to submit issues or pull requests.

## Usefull links

- [Docker Official website](https://docs.docker.com/engine/install/ubuntu)
- [Best Ros2 Docker youtube series  by Articulated Robotics](https://www.youtube.com/watch?v=dihfA7Ol6Mw&list=PLunhqkrRNRhaqt0UfFxxC_oj7jscss2qe&index=6)
- [automaticaddison ROS2 docker tutorial](https://automaticaddison.com/the-complete-guide-to-docker-for-ros-2-jazzy-projects/)
- [Guide for ROS and Docker](https://blog.robotair.io/the-complete-beginners-guide-to-using-docker-for-ros-2-deployment-2025-edition-0f259ca8b378)
- [Ros2 and gui setup](https://medium.com/ai-casts-blog/2-quick-ways-to-use-gui-with-ros-ros-2-docker-images-44c24057e147)
- [Uinversity of Ottawa docker setup](https://github.com/wail-uottawa/docker-ros2-elg5228)
- [My GitHub Repository](https://github.com/DavideCapuzz/ros2docker)


