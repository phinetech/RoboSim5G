# Use the OAI NR UE base image as the starting point
FROM oaisoftwarealliance/oai-nr-ue:2024.w44

ARG ROBOT_PROJECT_NAME ROS_DISCOVERY_SERVER
# Set environment variables for ROS 2 installation
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# Update package list and install dependencies
RUN apt-get update && \
    apt-get install -y \
    sudo \
    curl \
    gnupg2 \
    lsb-release \
    locales \
    build-essential \
    git \
    wget \
    software-properties-common \
    && rm -rf /var/lib/apt/lists/*

# Set up locale
RUN sudo locale-gen en_US.UTF-8 && \
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

ENV LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8

RUN add-apt-repository universe && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" |sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/ros2.list
# Install ROS 2 and other dependencies
RUN apt-get update && \
    apt upgrade -y && \
    apt-get install -y \
    ros-${ROS_DISTRO}-desktop \
    ros-dev-tools \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup && \
    rm -rf /var/lib/apt/lists/*

RUN  apt-get update --fix-missing
RUN  apt-get install -y libignition-msgs5-dev
RUN  apt-get install -y libignition-transport8-dev 
RUN  apt-get install -y libignition-tools-dev 
RUN  apt-get install -y ros-humble-ros-ign-bridge
RUN  apt-get install -y iptables

WORKDIR /app

COPY ./dds.xml .

COPY $ROBOT_PROJECT_NAME robot_logic
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    cd /app/robot_logic && \
    colcon build"
# Source the ROS setup file
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc
RUN echo "export ROS_DISCOVERY_SERVER="$ROS_DISCOVERY_SERVER"" >> ~/.bashrc
RUN echo "export IGN_IP=192.168.73.150" >> ~/.bashrc
RUN echo "source /app/robot_logic/install/setup.bash" >> ~/.bashrc
RUN echo "export FASTRTPS_DEFAULT_PROFILES_FILE=/app/dds.xml" >> ~/.bashrc


