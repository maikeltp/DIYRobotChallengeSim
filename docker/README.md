# ROS 2 Development Environment

Docker-based development environment for ROS 2 Jazzy. The environment includes essential ROS 2 packages, Gazebo simulation tools, and visualization capabilities using RViz2.

## Prerequisites

- Docker installed on your system
- X11 for GUI applications
- Git (optional, for version control)

## Files Overview

### `Dockerfile`
Defines the development environment container with:
- ROS 2 Jazzy as the base image
- Essential ROS 2 development tools and packages
- Navigation and visualization packages (Nav2, RViz2)
- Gazebo simulation integration
- User-specific configuration with proper permissions

### `build.sh`
Builds the Docker image with the current user's credentials:
```bash
./build.sh
```

### `start.sh`
Launches the development container with:
- X11 forwarding for GUI applications
- Host network access
- Mounted volumes for:
  - X11 socket
  - SSH configuration
  - ROS 2 workspace
- Proper working directory configuration

## Quick Start

1. Clone this repository:
```bash
git clone <repository-url>
cd <repository-name>
```

2. Make the scripts executable:
```bash
chmod +x build.sh start.sh
```

3. Build the Docker image:
```bash
./build.sh
```

4. Start the development environment:
Note: make sure to update the line `-v $HOME/autonomous-race-car:/home/devuser/ros2_ws \`  with the path where you cloned th repository.
```bash
./start.sh
```

Once inside the docker container you can build the project using the command:
```bash
colcon build
```

If you want to run a ROS2 python node, make sure the pyhton environment is activated and the PYTHONPATH includes the path to the python environment. Sourcing the `.bashrc` in the devuser space will take care of all those steps.
```bash
source /home/devuser/.bashrc
```

## Development Workspace

The container creates and configures a ROS 2 workspace at `/home/devuser/ros2_ws`. This workspace is mounted from your host system, allowing you to:
- Develop code on your host machine
- Build and test within the container

## Notes

- The environment uses privileged mode to access system devices (useful for hardware interactions)
- X11 forwarding is configured for GUI applications
- SSH configuration is mounted from the host for Git operations
- The container runs with the current user's UID/GID to avoid permission issues

## Customization

You can customize the environment by:
1. Modifying the `Dockerfile` to add additional packages
2. Adjusting the Colcon build defaults in the `.colcon/defaults.yaml` configuration
3. Modifying the volume mounts in `start.sh` for additional persistence needs

## Troubleshooting

If you encounter X11 forwarding issues:
1. Ensure X11 is installed and running on your host system
2. Check that the DISPLAY environment variable is set correctly
3. Verify that `xhost` commands execute successfully

For permission issues:
1. Verify that the USERNAME build argument matches your system username
2. Check that the UID/GID in the Dockerfile matches your user's IDs

Gazebo work is black or doesn't load
1. Sometimes the firewall conflicts with Gazebo. Try desabling it.
```bash
sudo ufw desable
```
