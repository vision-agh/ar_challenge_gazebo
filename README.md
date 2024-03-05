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

**NOTE:** Remember about dot before the path.

## Challenge overview

Challenge can be started with the prepared [challenge.launch](./launch/challenge.launch):

```
roslaunch ar_challenge challenge.launch
```

This script launches the Gazebo simulator, spawns Iris UAV model and runs the control node (`iris_node`).
You can communicate with control node via two topics:

- `/iris_control/pose` ([std_msgs/Bool](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Bool.html)) - node publishes current pose (position + orientation),
- `/iris_control/cmd_vel` ([geometry_msgs/Twist](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html)) - node subscribes to get the velocity control signal

Beside that, Iris UAV model provides video stream from camera and altitude from LiDAR sensor:

- `/iris/usb_cam/camera_info` ([sensor_msgs/CameraInfo](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/CameraInfo.html)) - camera parametres,
- `/iris/usb_cam/image_raw` ([sensor_msgs/Image](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html)) - RGB image,
- `/laser/scan` ([sensor_msgs/LaserScan](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/LaserScan.html)) - LiDAR measurement.

### Goal formulation

The main goal is to detect ArUco tag on the camera image and control the UAV to hover directly over the center of the tag.

### Initial conditions

Both UAV and landing pad are launched in predefined positions.
After that, the control node performs takeoff to predefined altitude and listens to messages coming in `/iris_control/cmd_vel` topic.

**NOTE 1:** Initial positions of UAV and landing pad as well as the takeoff altitude may vary during evaluation.
You can only assume that after the takeoff, landing pad will be inside the field of view of camera (will be visible in the image).

**NOTE 2:** The control node runs at 20 Hz.
Therefore your algorithm should also publish new velocity command on `/iris_control/cmd_vel` with the same frequency.

### Evaluation

Evaluation will consist of N runs with random initial conditions.
To achieve aforementioned goal, UAV has to hoover above the center of the ArUco tag.
This means that horizontal distance between UAV and ArUco tag should be less than 15 cm for at least 30 seconds.

## Useful links and tips
- [GitHub documentation](https://docs.github.com/en)
- [Gazebo simulator](https://gazebosim.org/docs)
- [PX4 Autopilot](https://px4.io/)
- [Docker containers](https://docs.docker.com/)
- [ROS documentation](http://wiki.ros.org/)
- [ROS tutorials](http://wiki.ros.org/ROS/Tutorials)
