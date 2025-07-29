FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# Install dependencies
RUN apt update 
RUN apt install -y ros-dev-tools
RUN apt install -y iputils-ping iproute2
# Required packages for building
# Install system dependencies
RUN apt update && apt install -y \
    git cmake build-essential \
    libasio-dev libtinyxml2-dev \
    libfoonathan-memory-dev \
    iputils-ping iproute2

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
