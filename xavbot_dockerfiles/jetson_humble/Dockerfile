# FROM dustynv/ros:humble-ros-base-l4t-r35.4.1
FROM ros:humble-ros-base

# Install xavbot dependencies
RUN apt-get update && apt-get install -y \
    libserial-dev \
    python3-pip \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-robot-localization \
    ros-${ROS_DISTRO}-spatio-temporal-voxel-layer

RUN pip install \
    pyserial \
    setuptools==58.2.0

# Install testing dependencies ros2
RUN apt-get update && apt-get install -y \
    x11-apps

