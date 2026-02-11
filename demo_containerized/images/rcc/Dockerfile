# Use the OAI NR UE base image as the starting point
FROM ubuntu:22.04

# Set environment variables for ROS 2 installation
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

RUN apt-get update && apt-get install -y curl gnupg2 lsb-release

# Fix GPG key issue
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/ros2.list
        # Install colcon
RUN apt-get update && \
    apt-get install -y \
    python3-colcon-common-extensions \
    build-essential \
    ros-humble-desktop 

RUN apt-get update && \
    apt-get install -y \
    libasio-dev libtinyxml2-dev \
    libfoonathan-memory-dev \
    iputils-ping iproute2 \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup && \
    rm -rf /var/lib/apt/lists/*



WORKDIR /app   
COPY rviz_rcc rviz_rcc
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    cd /app/rviz_rcc && \
    colcon build"

# Source the ROS setup file
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc
RUN echo "export ROS_DISCOVERY_SERVER="192.168.70.159:11811"" >> ~/.bashrc
RUN echo "source /app/rviz_rcc/install/setup.bash" >> ~/.bashrc

