FROM arm64v8/ros:humble-perception-jammy

RUN apt-get update \
   && apt-get -y install --no-install-recommends \
   ros-$ROS_DISTRO-dynamixel-workbench-toolbox \
   ros-$ROS_DISTRO-gripper-controllers \
   ros-$ROS_DISTRO-hardware-interface \
   ros-$ROS_DISTRO-joint-state-publisher-gui \
   ros-$ROS_DISTRO-moveit \
   ros-$ROS_DISTRO-realsense2-description \
   ros-$ROS_DISTRO-realtime-tools \
   ros-$ROS_DISTRO-ros2-control \
   ros-$ROS_DISTRO-ros2-controllers \
   ros-$ROS_DISTRO-teleop-twist-keyboard \
   ros-$ROS_DISTRO-xacro \
   libserial-dev \
   
   # Clean up
   && apt-get autoremove -y \
   && apt-get clean -y \
   && rm -rf /var/lib/apt/lists/*
ENV DEBIAN_FRONTEND=dialog

RUN apt-get update \
   && apt-get -y install --no-install-recommends python3-pip \
   && pip install setuptools==58.2.0

RUN echo "if [ -f ./install/setup.bash ]; then source ./install/setup.bash; fi" >> ~/.bashrc