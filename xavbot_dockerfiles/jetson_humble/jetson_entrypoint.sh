#!/bin/bash
set -e

# Setup ros environment
if [ -e "./install/setup.bash" ]; then
  source install/setup.bash
else
  source /opt/ros/$ROS_DISTRO/setup.bash
fi

# Optionally rebuild workspace
if [ "$REBUILD" = true ]; then
  colcon build --symlink-install \
  --packages-up-to xavbot_bringup sllidar_ros2 rf2o_laser_odometry \
  --packages-ignore xavbot_perception xavbot_teleop xavbot_interfaces
fi

# Add teleop alias
echo "alias teleop='ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/xavbot_platform_controller/reference -p stamped:=True -p frame_id:=base_link'" >> ~/.bash_aliases

exec "$@"