#!/bin/bash
#
# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

# Build ROS dependency
echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
source /opt/ros/${ROS_DISTRO}/setup.bash

export FASTRTPS_DEFAULT_PROFILES_FILE=/workspaces/isaac_ros-dev/src/xavbot/FASTRTPS_PROFILE.xml
echo 'alias cbsi="colcon build --symlink-install"' >> ~/.bash_aliases
echo 'alias bringup=". /workspaces/isaac_ros-dev/install/setup.bash && ros2 launch xavbot_bringup xavbot.launch.py"' >> ~/.bash_aliases

# sudo apt-get update
# rosdep update

# Restart udev daemon
sudo service udev restart

$@
