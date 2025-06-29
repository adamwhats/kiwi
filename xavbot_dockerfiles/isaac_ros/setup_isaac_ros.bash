#!/bin/bash

if [ -z $ISAAC_ROS_WS ]; then
    echo "Could not find ISAAC_ROS_WS environment variable, assuming /home/$USER/workspaces/isaac_ros-dev"
    ISAAC_ROS_WS="/home/$USER/workspaces/isaac_ros-dev"
fi

OVERRIDE_DIR=$(dirname "$0")
ISAAC_ROS_COMMON_DIR=$ISAAC_ROS_WS/src/isaac_ros_common

cp $OVERRIDE_DIR/.isaac_ros_common-config $ISAAC_ROS_COMMON_DIR/scripts/
cp $OVERRIDE_DIR/.isaac_ros_dev-dockerargs $ISAAC_ROS_COMMON_DIR/scripts/
cp $OVERRIDE_DIR/workspace-entrypoint.sh $ISAAC_ROS_COMMON_DIR/docker/scripts/