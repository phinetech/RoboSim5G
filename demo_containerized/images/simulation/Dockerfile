# Use Ubuntu 22.04 as the starting point
FROM ubuntu:22.04

# Set environment variables to avoid interactive prompts
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# Install base dependencies, CA certificates
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    gnupg2 \
    lsb-release \
    ca-certificates \
    # Clean up apt cache in the same RUN instruction
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y net-tools iproute2 iputils-ping iperf3

RUN apt-get update && apt-get install -y \
    curl \
    && curl -fsSL https://get.docker.com -o get-docker.sh \
    && sh get-docker.sh
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

# Add Gazebo repository
RUN curl -sSL http://packages.osrfoundation.org/gazebo.key | \
    gpg --dearmor -o /usr/share/keyrings/gazebo-archive-keyring.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list

# Install ROS 2 Desktop, Gazebo Fortress, ROS-Gazebo integration, build tools, and your specific dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    # ROS 2 Core + Tools
    ros-humble-desktop \
    python3-colcon-common-extensions \
    build-essential \
    ros-humble-ros-gz \
    ros-humble-ros-ign-gazebo \
    ros-humble-ros-ign-bridge \
    ros-humble-ros2-control \
    ros-humble-controller-manager\
    ros-humble-gz-ros2-control \
    ros-humble-joint-state-broadcaster \
    ros-humble-joint-trajectory-controller \
    ros-humble-velocity-controllers \ 
    # Gazebo Fortress (Ignition 6)
    ignition-fortress \
    # Clean up apt cache
    && rm -rf /var/lib/apt/lists/*

# Set up the working directory
WORKDIR /app

# Copy your workspace source code
COPY gazebo_launch gazebo_launch
COPY nav_stack nav_stack
COPY dds.xml dds.xml
COPY oai_setup oai_setup
COPY images images

# Build your workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    cd /app/gazebo_launch && \
    colcon build"

# Set up the entry point environment by modifying .bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc && \
    echo "source /app/gazebo_launch/install/setup.bash" >> ~/.bashrc && \
    echo "export IGN_IP=192.168.73.151">>  ~/.bashrc && \
    echo "export FASTRTPS_DEFAULT_PROFILES_FILE=/app/dds.xml" >> ~/.bashrc \
    echo "export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/ign-gazebo-6/plugins" >> ~/.bashrc

# Optional: Set a default command or entrypoint
# CMD ["/bin/bash"]