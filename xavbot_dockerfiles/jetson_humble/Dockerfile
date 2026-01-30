FROM dustynv/ros:humble-ros-base-l4t-r36.3.0

# Update ROS apt repo keyring
RUN mkdir -p /etc/apt/keyrings && \
    wget -O /etc/apt/keyrings/ros-archive-keyring.gpg https://raw.githubusercontent.com/ros/rosdistro/master/ros.key && \
    rm -f /etc/apt/sources.list.d/ros* /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2.list

# Configure pip to use correct Jetson PyPI server
ENV PIP_INDEX_URL=https://pypi.jetson-ai-lab.io/jp6/cu126

# Install dependencies
RUN apt-get update && apt-get install -y --no-install-recommends --fix-broken \
    libserial-dev \
    python3-pip \
    ros-${ROS_DISTRO}-control-msgs \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-diagnostic-updater \
    ros-${ROS_DISTRO}-foxglove-bridge \
    ros-${ROS_DISTRO}-moveit-msgs \
    ros-${ROS_DISTRO}-nav2-msgs \
    ros-${ROS_DISTRO}-realsense2-camera-msgs \
    ros-${ROS_DISTRO}-robot-localization \
    ros-${ROS_DISTRO}-std-srvs \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

RUN pip install \
    pyserial \
    setuptools==58.2.0

# Build realsense-ros
ENV CMAKE_PREFIX_PATH=${ROS_ROOT}:$CMAKE_PREFIX_PATH

RUN curl -sSf https://librealsense.realsenseai.com/Debian/librealsenseai.asc | \
        gpg --dearmor | sudo tee /etc/apt/keyrings/librealsenseai.gpg > /dev/null && \
    echo "deb [signed-by=/etc/apt/keyrings/librealsenseai.gpg] https://librealsense.realsenseai.com/Debian/apt-repo `lsb_release -cs` main" | \
        sudo tee /etc/apt/sources.list.d/librealsense.list && \
    apt-get update && \
    apt-get install -y librealsense2-utils librealsense2-dev && \
    mkdir -p /opt/realsense_ws && cd /opt/realsense_ws && \
    git clone https://github.com/IntelRealSense/realsense-ros.git -b r/4.56.4 src && \
    . /opt/ros/${ROS_DISTRO}/install/setup.bash && \
    colcon build --packages-up-to realsense2_camera --cmake-args -DCMAKE_BUILD_TYPE=Release --install-base ${ROS_ROOT}/install --merge-install

# Build rplidar_ros2
RUN mkdir /opt/rplidar_ros2 && cd /opt/rplidar_ros2 && \
    git clone https://github.com/frozenreboot/rplidar_ros2_driver.git src && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    . /opt/ros/${ROS_DISTRO}/install/setup.bash && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --install-base ${ROS_ROOT}/install --merge-install

RUN echo "if [ -f ./install/setup.bash ]; then source ./install/setup.bash; fi" >> ~/.bashrc

RUN echo "alias pi-deploy='bash \$(find /xavbot_ws -name pi_deploy.sh -type f | head -n 1)'" >> ~/.bash_aliases
RUN echo "alias launch='colcon build --packages-up-to xavbot_bringup && source ./install/setup.bash && ros2 launch xavbot_bringup jetson.launch.py'" >> ~/.bash_aliases