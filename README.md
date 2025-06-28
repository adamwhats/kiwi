# XavBot

My journey into building a robot from scratch, with the initial aim of exploring the Nav2 framework. Future goals are experimenting with building my own SLAM and perception algorithms as well as Gazebo/ Isaac Sim simulation.

XavBot is a small holonomic robot based around the NVIDIA Jetson Xavier NX with ab Intel RealSense D435i RGB-D camera, RPLidar A1M8 lidar & Pimoroni Motor2040 motor control board. 

# Packages

| Name | Description |
| :----: | --- |
|[`xavbot_bringup`](https://github.com/adamwhats/xavbot/tree/main/xavbot_description)|Contains the launch and configuration files.|
|[`xavbot_controller`](https://github.com/adamwhats/xavbot/tree/main/xavbot_controller)|The kinematic controller for mecanum wheels.|
|[`xavbot_description`](https://github.com/adamwhats/xavbot/tree/main/xavbot_description)|URDF xacros that describe the physical geometry and ros2_control interfaces.|
|[`xavbot_dockerfiles`](https://github.com/adamwhats/xavbot/tree/main/xavbot_dockerfiles)|Dockerfiles for both xavbot itself and teleop on a remote machine.|
|[`xavbot_hardware`](https://github.com/adamwhats/xavbot/tree/main/xavbot_hardware)|Hardware interface for driving the Motor2040 board. Written with lots of guidance from the excellent series by [Articulated Robotics](https://www.youtube.com/c/ArticulatedRobotics).|
|[`xavbot_teleop`](https://github.com/adamwhats/xavbot/tree/main/xavbot_teleop)|A launch file and rviz config for operating xavbot with a dualshock 4 controller (TODO).|

## Setup Notes
### Iptables Rules
 TODO - Implement method of automatically applying the rules at /etc/iptables/rules.v4 on the jetson

### Isaac ROS Dev Environment
This codebase is designed to work in the [NVIDIA Isaac ROS Docker Development Environment](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_common/index.html). To set this up, the original files found at:
  - `src/isaac_ros_common/scripts/.isaac_ros_common-config`
  - `src/isaac_ros_common/scripts/.isaac_ros_dev-dockerargs`
  - `src/isaac_ros_common/docker/scripts/workspace-entrypoint.sh`
  
  need to be replaced with their counterparts from inside the `/home/dev/workspaces/isaac_ros-dev/src/xavbot/xavbot_dockerfiles/isaac_ros` directory

## Bringup
You can then bring up the system with:
"""
./src/isaac_ros_common/scripts/run_dev.sh 
"""