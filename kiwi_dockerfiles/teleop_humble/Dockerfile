FROM althack/ros2:humble-dev 

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
   && apt-get -y install --no-install-recommends ros-$ROS_DISTRO-rviz2 \
   ros-$ROS_DISTRO-image-transport-plugins \
   ros-$ROS_DISTRO-joint-state-publisher-gui \
   ros-$ROS_DISTRO-teleop-twist-keyboard \
   ros-$ROS_DISTRO-xacro \
   # Temporary imports for kiwi_arm development
   # ros-$ROS_DISTRO-behaviourtree-cpp \
   ros-$ROS_DISTRO-dynamixel-workbench-toolbox \
   ros-$ROS_DISTRO-gripper-controllers \
   ros-$ROS_DISTRO-hardware-interface \
   ros-$ROS_DISTRO-moveit \
   ros-$ROS_DISTRO-navigation2 \
   ros-$ROS_DISTRO-nav2-bringup \
   ros-$ROS_DISTRO-ros2-control \
   ros-$ROS_DISTRO-ros2-controllers \
   ros-$ROS_DISTRO-realtime-tools \
   ros-$ROS_DISTRO-realsense2-camera \
   ros-$ROS_DISTRO-realsense2-description \
   libserial-dev \
   #
   # Clean up
   && apt-get autoremove -y \
   && apt-get clean -y \
   && rm -rf /var/lib/apt/lists/*
ENV DEBIAN_FRONTEND=dialog

RUN pip install setuptools==58.2.0

# Set up auto-source of workspace for ros user
RUN sudo usermod -aG dialout ros
ARG WORKSPACE
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc

# Configure Fast DDS to allow discovery on the kiwi subnet
ENV FASTRTPS_DEFAULT_PROFILES_FILE /workspaces/kiwi_local_ws/src/kiwi/FASTRTPS_PROFILE.xml