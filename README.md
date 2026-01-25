# XavBot

My journey into building a robot from scratch, with the initial aim of exploring the Nav2 framework. Future goals are experimenting with building my own SLAM and perception algorithms.

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
### Jetson <-> Pi connection
Setup a local wire connection between the jetson and pi

On jetson:
```
# Remove old connections
sudo nmcli connection delete jetson-pi
# Create shared connection
sudo nmcli connection add type ethernet con-name jetson-pi ifname enP8p1s0 \
  ipv4.method shared ipv4.addresses 10.42.0.1/24
# Activate
sudo nmcli connection up jetson-pi
```

On pi:
```
# Remove old connections
sudo nmcli connection delete jetson-pi
# Create shared connection
sudo nmcli connection add type ethernet con-name jetson-pi ifname eth0 \
  ipv4.method manual \
  ipv4.addresses 10.42.0.2/24 \
  ipv4.gateway 10.42.0.1 \
  ipv4.dns "8.8.8.8,8.8.4.4"
# Activate
sudo nmcli connection up jetson-pi
```