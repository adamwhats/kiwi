FROM dustynv/ros:humble-ros-base-l4t-r36.3.0

# Update ROS apt repo keyring
RUN mkdir -p /etc/apt/keyrings && \
    wget -O /etc/apt/keyrings/ros-archive-keyring.gpg https://raw.githubusercontent.com/ros/rosdistro/master/ros.key && \
    rm -f /etc/apt/sources.list.d/ros* /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2.list

# Configure pip to use correct Jetson PyPI server
ENV PIP_INDEX_URL=https://pypi.jetson-ai-lab.io/jp6/cu126

# Build librealsense with LibUVC backend, necessary to get IMU working on JP6
RUN mkdir -p /etc/udev/rules.d && \
    cd /root && \
    git clone https://github.com/realsenseai/librealsense.git && \
    cd librealsense && \
    chmod +x scripts/libuvc_installation.sh && \
    echo "" | ./scripts/libuvc_installation.sh && \
    cd / && rm -rf /root/librealsense_build /root/librealsense

# Build realsense-ros
ENV CMAKE_PREFIX_PATH=${ROS_ROOT}:$CMAKE_PREFIX_PATH

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-diagnostic-updater && \
    apt-get clean && rm -rf /var/lib/apt/lists/* && \
    mkdir -p /opt/realsense_ros && cd /opt/realsense_ros && \
    git clone https://github.com/IntelRealSense/realsense-ros.git -b r/4.56.4 src && \
    . /opt/ros/${ROS_DISTRO}/install/setup.bash && \
    colcon build --packages-up-to realsense2_camera --cmake-args -DCMAKE_BUILD_TYPE=Release --install-base ${ROS_ROOT}/install --merge-install

# # Build OpenVINS
# RUN apt-get update && apt-get install -y --no-install-recommends \
#     libceres-dev libeigen3-dev libboost-all-dev && \
#     apt-get clean && rm -rf /var/lib/apt/lists/* && \
#     mkdir -p /opt/openvins && cd /opt/openvins && \
#     git clone https://github.com/rpng/open_vins.git src && \
#     . /opt/ros/${ROS_DISTRO}/install/setup.bash && \
#     colcon build --packages-up-to ov_msckf --parallel-workers 1 --cmake-args -DCMAKE_BUILD_TYPE=Release --install-base ${ROS_ROOT}/install --merge-install

# Build rplidar_ros2
RUN mkdir -p /opt/rplidar_ros2 && cd /opt/rplidar_ros2 && \
    git clone https://github.com/frozenreboot/rplidar_ros2_driver.git src && \
    . /opt/ros/${ROS_DISTRO}/install/setup.bash && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --install-base ${ROS_ROOT}/install --merge-install

# Build rf2o_laser_odometry
RUN mkdir -p /opt/rf2o_laser_odometry && cd /opt/rf2o_laser_odometry && \
    git clone https://github.com/MAPIRlab/rf2o_laser_odometry.git src && \
    . /opt/ros/${ROS_DISTRO}/install/setup.bash && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --install-base ${ROS_ROOT}/install --merge-install

# Install other local dependencies
RUN apt-get update && apt-get install -y --no-install-recommends --fix-broken \
    libserial-dev \
    python3-pip \
    ros-${ROS_DISTRO}-control-msgs \
    ros-${ROS_DISTRO}-dynamixel-interfaces \
    ros-${ROS_DISTRO}-foxglove-bridge \
    ros-${ROS_DISTRO}-moveit-msgs \
    ros-${ROS_DISTRO}-nav2-msgs \
    ros-${ROS_DISTRO}-realsense2-camera-msgs \
    ros-${ROS_DISTRO}-robot-localization \
    ros-${ROS_DISTRO}-std-srvs \
    rsync \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

RUN pip install \
    pyserial \
    setuptools==58.2.0

# Bash helpers
RUN echo "if [ -f ./install/setup.bash ]; then source ./install/setup.bash; fi" >> ~/.bashrc
RUN echo "alias pi-deploy='bash \$(find /xavbot_ws -name pi_deploy.sh -type f | head -n 1)'" >> ~/.bash_aliases
RUN echo "alias launch='colcon build --packages-up-to xavbot_bringup && source ./install/setup.bash && ros2 launch xavbot_bringup jetson.launch.py'" >> ~/.bash_aliases