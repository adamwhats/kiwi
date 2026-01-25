FROM dustynv/ros:humble-ros-base-l4t-r36.3.0

# Update ROS apt repo keyring
RUN mkdir -p /etc/apt/keyrings && \
    wget -O /etc/apt/keyrings/ros-archive-keyring.gpg https://raw.githubusercontent.com/ros/rosdistro/master/ros.key && \
    rm -f /etc/apt/sources.list.d/ros* /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2.list

# Install dependencies
RUN apt-get update && apt-get install -y --no-install-recommends --fix-broken \
    libserial-dev \
    python3-pip \
    ros-${ROS_DISTRO}-nav2-msgs \
    ros-${ROS_DISTRO}-robot-localization \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Configure pip to use correct Jetson PyPI server
ENV PIP_INDEX_URL=https://pypi.jetson-ai-lab.io/jp6/cu126

RUN pip install \
    pyserial \
    setuptools==58.2.0

# Install testing dependencies
RUN apt-get update && apt-get install -y \
    x11-apps \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

RUN echo "if [ -f ./install/setup.bash ]; then source ./install/setup.bash; fi" >> ~/.bashrc

