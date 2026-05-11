FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# Install dependencies
RUN apt update 
RUN apt install -y ros-dev-tools
RUN apt install -y iputils-ping iproute2
# Required packages for building
# Install system dependencies with retry logic for mirror sync issues
RUN apt-get clean && \
    for i in 1 2 3; do \
        apt-get update && break || sleep 10; \
    done && \
    apt-get install -y \
    git cmake build-essential \
    libasio-dev libtinyxml2-dev \
    libfoonathan-memory-dev \
    iputils-ping iproute2 && \
    rm -rf /var/lib/apt/lists/*

# --- Install Fast-CDR ---
WORKDIR /opt
RUN git clone --depth 1 https://github.com/eProsima/Fast-CDR.git && \
    mkdir -p Fast-CDR/build && \
    cd Fast-CDR/build && \
    cmake .. && \
    make -j$(nproc) && \
    make install

# --- Install Fast DDS ---
RUN git clone --depth 1 https://github.com/eProsima/Fast-DDS.git && \
    mkdir -p Fast-DDS/build && \
    cd Fast-DDS/build && \
    cmake .. && \
    make -j$(nproc) && \
    make install

RUN which fastdds
# Optional: verify fastdds is in PATH
# ROS setup
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc

ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
WORKDIR /app
