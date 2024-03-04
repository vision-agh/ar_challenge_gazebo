# Aerial Robotics 2024 Gazebo Challenge

This repository containes all files necessary to compete in the Gazebo Challenge during Aerial Robotics course (summer 2024).

## Startup

### Prerequisities

The repository is prepared to use it inside the [Docker](https://www.docker.com/) container running under the Ubuntu OS (22.04 preferably).

**NOTE:** It is important to use the Ubuntu OS because of the GUI setup, which should be resolved differently with Windows.

To properly setup Docker on your computer do the following steps:

1. [Install](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository) Docker Engine.
2. [Configure](https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user) Docker to use as a non-root user.
3. (Optionally, but recommended) [Install](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) Nvidia Container Toolkit (without experimental packages and without rootless mode).

### Container building

To build the container you only need to run the [docker_build.sh](./docker_build.sh) script:

```
./docker_build.sh
```

Above script build the container described in the [Dockerfile](./.devcontainer/Dockerfile).
It is based on two images:

1. [Nvidia OpenGL Runtime](https://hub.docker.com/r/nvidia/opengl/tags)
2. [PX4 with ROS Noetic](https://hub.docker.com/r/px4io/px4-dev-ros-noetic)

The first one is necessary for Gazebo GUI, while the second one provides all prerequisities for PX4 stack (UAV control), including ROS Noetic packages.
Based on them, provided Dockerfile installs [Eigen](https://eigen.tuxfamily.org) library, builds [PX4 stack](https://github.com/PX4/PX4-Autopilot), some additional packages to facilitates working with ROS and builds the catkin workspace.

### Container running

To run the built container you only need to run the [docker_run.sh](./docker_run.sh) script:

```
./docker_run.sh
```

Above script starts the container or attaches to running one, with some additional arguments (you can check them - most of them are necessary to provide GUI connection between Docker and Ubuntu OS).

**NOTE:** This script mounts also the local directory to the src directory of the catkin workspace inside Docker container.
Therefore all changes made locally are visible inside Docker containter, which is useful during development.

When above script ends, terminal is automatically attached to the container.
Before launching anything you need to re-build the catkin workspace (to include this package):

```
cd ~/sim_ws
catkin build
```

Next you need to update some paths with [environment_vars.sh] script:

```
. ~/sim_ws/src/ar_challenge/environment_vars.sh
```

**NOTE:** Remember about dot before path.

## Challenge overview

## ROS basics

## Useful links and tips
