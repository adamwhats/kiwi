#!/bin/bash
set -e

# Setup ros environment
if [ -e "./install/setup.bash" ]; then
  source  "./install/setup.bash"
else
  source "/opt/ros/$ROS_DISTRO/setup.bash"
fi

# Optionally rebuild workspace
if [ "$REBUILD" = true ]; then
  colcon build --symlink-install --packages-ignore xavbot_behaviour xavbot_perception xavbot_teleop
  source "./install/setup.bash"
fi

exec "$@"
