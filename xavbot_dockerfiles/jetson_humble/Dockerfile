FROM nvcr.io/nvidia/l4t-cuda:12.6.11-runtime

ENV DEBIAN_FRONTEND=noninteractive
ENV PIP_INDEX_URL=https://pypi.jetson-ai-lab.io/jp6/cu126

# Build librealsense with LibUVC backend, required for IMU support on JetPack 6
RUN apt-get update && apt-get install -y --no-install-recommends \
    git cmake build-essential wget libusb-1.0-0-dev libglfw3-dev pkg-config lsb-release sudo && \
    mkdir -p /etc/udev/rules.d && \
    cd /root && \
    git clone https://github.com/IntelRealSense/librealsense.git && \
    cd librealsense && \
    chmod +x scripts/libuvc_installation.sh && \
    echo "" | ./scripts/libuvc_installation.sh && \
    cd / && rm -rf /root/librealsense_build /root/librealsense && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# Install ROS2 Humble
ENV ROS_DISTRO=humble
SHELL ["/bin/bash", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl gnupg2 lsb-release software-properties-common && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-ros-base \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# Build ROS2 source dependencies
ARG DEPS_DIR=/opt/xavbot_deps
RUN mkdir -p ${DEPS_DIR}/src && cd ${DEPS_DIR} && \
    git clone https://github.com/IntelRealSense/realsense-ros.git -b r/4.56.4 src/realsense-ros && \
    git clone https://github.com/frozenreboot/rplidar_ros2_driver.git src/rplidar && \
    git clone https://github.com/MAPIRlab/rf2o_laser_odometry.git src/rf2o && \
    apt-get update && apt-get install -y --no-install-recommends libboost-dev && \
    rosdep init && rosdep update --rosdistro=${ROS_DISTRO} && \
    rosdep install --from-paths src --ignore-src -r -y --skip-keys="librealsense2" && \
    . /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --merge-install && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# Install other dependencies
RUN apt-get update && \
    apt-mark hold *realsense* && \    
    apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-control-msgs \
    ros-${ROS_DISTRO}-dynamixel-interfaces \
    ros-${ROS_DISTRO}-foxglove-bridge \
    ros-${ROS_DISTRO}-robot-localization \
    ros-${ROS_DISTRO}-rtabmap-slam \
    rsync && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# Bash helpers
RUN echo "source ${DEPS_DIR}/install/setup.bash" >> ~/.bashrc && \
    echo "alias pi-deploy='bash \$(find /xavbot_ws -name pi_deploy.sh -type f | head -n 1)'" >> ~/.bash_aliases && \
    echo "alias launch='cd /xavbot_ws && colcon build --packages-up-to xavbot_bringup && source install/setup.bash && clear && ros2 launch xavbot_bringup jetson.launch.py'" >> ~/.bash_aliases

WORKDIR /xavbot_ws
