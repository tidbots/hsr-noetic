# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Docker-based development environment for HSR (Human Support Robot) robotics projects using ROS1 Noetic. The environment runs on NVIDIA CUDA 11.3 with Ubuntu 20.04 inside Docker containers.

## Build and Run Commands

```bash
# Build the Docker image
docker compose build

# Start the container
docker compose up

# Access running container from another terminal
docker compose exec hsr_noetic /bin/bash
```

## Development Modes

**Simulator mode** (inside container):
```bash
sim_mode  # Sets ROS to use localhost
roslaunch hsrb_gazebo_launch hsrb_megaweb2015_world.launch robot_name:=hsrc
```

**Note**: The `robot_name:=hsrc` (or `robot_name:=hsrb`) argument is required. The `.env` file sets `ROBOT_NAME` to your physical robot's hostname (e.g., `hsrc28`), but the simulator only supports generic model types (`hsrc` or `hsrb`). Without this argument, the launch will fail with "No such file or directory" for the URDF file.

**Physical robot mode**: Container automatically configures ROS_MASTER_URI to the robot hostname defined in `.env` (ROBOT_NAME). Requires chrony time synchronization between host PC and robot.

## Key Environment Configuration

Copy `.env.example` to `.env` and edit before building:
```bash
cp .env.example .env
```

Configuration variables:
- `USER_NAME`, `GROUP_NAME`, `UID`, `GID`: Container user settings
- `PASSWORD`: Container user password
- `WORKSPACE_DIR`: Shared directory between host and container (default: `/home/roboworks/share`)
- `NETWORK_IF`: Ethernet interface for robot connectivity (check with `ip link show`)
- `ROBOT_NAME`: Robot hostname (e.g., `hsrc28`)

**Important**: Create the shared directory on the host before first run to avoid permission issues:
```bash
mkdir ~/share
```

## Architecture

- **Dockerfile**: Multi-stage build installing ROS Noetic, HSR packages from Toyota's private repository, and configuring the catkin workspace
- **compose.yaml**: GPU passthrough, X11 forwarding, host networking, audio device access
- **assets/entrypoint.sh**: Container startup script sourcing ROS setup
- **scripts/**: Python demo scripts using `hsrb_interface` library
- **sample/hsrb_samples/**: Git submodule containing Toyota's official HSR sample code (must run `git submodule update --init` to populate)

## Catkin Workspace

Located at `~/catkin_ros/` inside the container. Sample packages are copied here during image build and built with `catkin_make`. Source both ROS and workspace setup:
```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ros/devel/setup.bash
```

## Running Sample Programs

Guide demo (navigation with TTS):
```bash
cd ~/scripts
python3 guide_mega.py
```

Grasping demo (AR marker recognition):
```bash
roslaunch spawn_bottle_with_marker.launch  # Spawn AR-marked object in Gazebo
python3 grasp_bottle.py
```

ROS interface samples:
```bash
rosrun hsrb_motion_samples head_message.py
```

## External Documentation

- HSR Development Manual: https://docs.hsr.io/hsr_develop_manual/
- HSR Python Interface: https://docs.hsr.io/hsr_develop_manual/python_interface/
- HSR ROS Interface: https://docs.hsr.io/hsr_develop_manual/ros_interface/
- HSRB User Manual: https://docs.hsr.io/hsrb_user_manual/
- HSRC User Manual: https://docs.hsr.io/hsrc_user_manual/
