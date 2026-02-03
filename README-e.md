# Developing HSR with ROS1 Noetic
Easy-to-use Docker environment for ROS Noetic Ninjemys.

- ROS Noetic Ninjemys
- Ubuntu 20.04 LTS (Focal Fossa) GPU
- GPU Version

## Prerequisites
Install NVIDIA (GPU) related settings, Docker, and Docker Compose on your computer.

The following host PC environment has been tested:

- Ubuntu 22.04.3 LTS
- Docker version 24.0.7, build afdd53b
- Docker Compose version v2.21.0

### Installing Docker and Docker Compose
### NVIDIA (GPU) Configuration

### Time Synchronization Between Robot (Physical) and Development PC
To operate the physical robot, time synchronization between the robot and the development PC is required.

Install chrony on the host PC (Ubuntu running on the development PC) to synchronize time with the physical robot.

Since the container uses the host PC's network, if the host PC is time-synchronized with the robot, the container can also maintain time synchronization.

Please follow the [HSR User Manual - Time Synchronization](https://docs.hsr.io/hsrc_user_manual/howto/pc_setup_for_robot.html#id2) for chrony time synchronization instructions.

This step is not necessary when developing with the simulator.

## Installation
```
$ cd ~
$ git clone https://github.com/tidbots/hsr-noetic.git
$ cd hsr-noetic
$ docker compose build
```
The image will be created with the name hsr:noetic-nvidia.
If you want to change the image name, edit compose.yaml.

## Editing Configuration Files
Edit the .env file according to your environment.

## Important Notes Before First Run
Directory creation, file editing, package installation, etc. performed in the Docker container will not be reflected in the Docker image and will be lost when the container is destroyed.
Think of it like classes (Docker image) and instances (Docker container) in object-oriented programming.

Therefore, we mount and share a directory from the host computer to the container.
This way, editing work within the mounted directory will not be lost.

### Creating a Shared Directory Between Host Computer and Container
Before starting the container for the first time, create a directory on the host computer as follows:
```
$ cd ~
$ mkdir share
```
Here we are creating a directory named "share".
The name of the shared directory is specified in the .env file as follows. If you want to use a different name, edit the .env file:
```
WORKSPACE_DIR=/home/roboworks/share
```

### If You Did Not Create the Shared Directory in Advance
If you did not create the shared directory in advance, the Docker system will automatically create it.
However, the automatically created shared directory will be created with root privileges, making it a directory that cannot be freely written to.
In such cases, after stopping the container, change the user and group of the directory you want to share on the host computer as follows:
```
$ cd ~
$ ls -al share
total 8
drwxr-xr-x  2 root   root     4096 Dec 31 23:29 .
drwxr-x--- 24 username groupname 4096 Dec 31 23:29 ..
```

```
$ sudo chown username share/
$ sudo chgrp groupname share/
```
Verify that the user and group names of the shared directory have been changed:
```
$ ls -al share
total 8
drwxr-xr-x  2 username groupname 4096 Dec 31 23:31 .
drwxr-x--- 24 username groupname 4096 Dec 31 23:31 ..
```

## Running the Container
Start the container with the following command:
```
$ cd ~/hsr-noetic
$ docker compose up
```
To enter the running container from another terminal on the host computer, execute:
```
$ cd ~/hsr-noetic
$ docker compose exec hsr-noetic /bin/bash
```

## Developing with the Simulator
### Starting the Simulator
```
$ sim_mode
$ roslaunch hsrb_gazebo_launch hsrb_megaweb2015_world.launch
```
![](/fig/Screenshot_from_2023-09-24_11-26-28__1_.png)

![](/fig/Screenshot_from_2023-09-24_11-26-44__1_.png)


### Venue Guide Sample
https://docs.hsr.io/hsr_develop_manual/python_interface/hsr_guide_sample.html#id2

When you run the sample, the robot will tour and guide you around the venue.
```
cd ~/roboworks/scripts
python3 guide_mega.py
```

### Grasping Sample
https://docs.hsr.io/hsr_develop_manual/python_interface/hsr_pick_sample.html

This is a sample that performs grasping using marker recognition.

1. Spawn an object with AR marker
See [AR Marker](https://docs.hsr.io/hsr_develop_manual/python_interface/marker_interface.html#spawn-bottle-with-marker-label)

```
roslaunch spawn_bottle_with_marker.launch
```
A bottle with a marker should appear in Gazebo as shown below.

![](/fig/marker_gazebo.png)

2. Running the grasping program
```
python3 grasp_bottle.py
```

### Using the ROS Interface
See https://docs.hsr.io/hsr_develop_manual/ros_interface/ros_interface.html

Sample programs are located in ~/catkin_ros/src/

For example, to run the head movement sample, execute the following command as described at https://docs.hsr.io/hsr_develop_manual/ros_interface/ros_controller_head.html:
```
rosrun hsrb_motion_samples head_message.py
```




## Developing with the Physical Robot


### Verifying Time Synchronization
Verify that the time is synchronized between your PC and the HSR.
```
$
```

```
$ ihsrb
```

### Verifying Connection to the Physical Robot
```
$ ping hsrb.local

```
### Verify that ROS-related environment variables are correctly set.
```
$ env | grep ROS
ROS_MASTER_URI=http://hsrb.local:11311
ROS_IP=192.168.190.20
```



### Try the Sample Programs (Toyota Official)
```
$ rosrun
```



## Advancing HSR Development
Please enjoy HSR programming by referring to the [HSR Development Manual (Latest)](https://docs.hsr.io/hsr_develop_manual/index.html).

## Advanced Usage
If you want to modify the development environment, please refer to each HSR user manual:
- [HSRB User Manual](https://docs.hsr.io/hsrb_user_manual/index.html)
- [HSRC User Manual](https://docs.hsr.io/hsrc_user_manual/index.html)
