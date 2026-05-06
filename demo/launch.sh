#!/bin/bash
# Unified launch script for RoboSim5G native demo
# Usage: CORE_NETWORK=oai|free5gc|open5gs ./launch.sh

set -e

CORE_NETWORK=${CORE_NETWORK:-oai}
REPO_ROOT=$(cd "$(dirname "$0")/.." && pwd)
DEMO_DIR=$(cd "$(dirname "$0")" && pwd)
CN_DIR="$REPO_ROOT/core_network_setup/$CORE_NETWORK"

# Validate CORE_NETWORK
if [[ ! "$CORE_NETWORK" =~ ^(oai|free5gc|open5gs)$ ]]; then
    echo "ERROR: CORE_NETWORK must be one of: oai, free5gc, open5gs (got: $CORE_NETWORK)"
    exit 1
fi

echo "=== Starting RoboSim5G native demo with $CORE_NETWORK core network ==="

# Set CN-specific environment variables
case $CORE_NETWORK in
    oai)
        export UE_ROUTE_SUBNET="10.0.0.0/24"
        export UE_ROUTE_GW="192.168.70.134"
        export UE_CARRIER_FREQ="3619200000"
        HOST_ROUTE_SUBNET="10.0.0.0/24"
        HOST_ROUTE_GW="192.168.70.134"
        ;;
    free5gc)
        export UE_ROUTE_SUBNET="10.0.0.0/24"
        export UE_ROUTE_GW="192.168.70.135"
        export UE_CARRIER_FREQ="3319680000"
        HOST_ROUTE_SUBNET="10.0.0.0/24"
        HOST_ROUTE_GW="192.168.70.135"
        ;;
    open5gs)
        export UE_ROUTE_SUBNET="10.0.0.0/24"
        export UE_ROUTE_GW="192.168.70.134"
        export UE_CARRIER_FREQ="3319680000"
        HOST_ROUTE_SUBNET="10.0.0.0/24"
        HOST_ROUTE_GW="192.168.70.134"
        ;;
esac

# Set common environment variables
export PROJECT_PATH="$DEMO_DIR"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-10}
export IGN_PARTITION=${IGN_PARTITION:-domain1}
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=~/workspace/install/lib
export ROS_DISCOVERY_SERVER="192.168.70.159:11811"
export GNB_NAME_FOR_BUTTON=${GNB_NAME_FOR_BUTTON:-"gNB1"}
export UE_NAME_FOR_BUTTON=${UE_NAME_FOR_BUTTON:-"ue_turtlebot"}
source "$DEMO_DIR/gazebo_launch/install/setup.bash"
export FASTRTPS_DEFAULT_PROFILES_FILE="$DEMO_DIR/dds.xml"
export NAME_ROBOT_1=${NAME_ROBOT_1:-"ue_turtlebot"}

# 1. Start the core network
echo "Starting $CORE_NETWORK core network..."
cd "$CN_DIR"
docker compose up -d

# 2. Wait for core network readiness
if [ "$CORE_NETWORK" = "oai" ]; then
    ./wait_for_core_network.sh
else
    sleep 10
fi

# 3. Add UE subscriber (open5gs only, after DB is ready)
if [ "$CORE_NETWORK" = "open5gs" ]; then
    echo "Adding UE subscriber to Open5GS database..."
    cd "$CN_DIR/scripts"
    ./add_ue_subscriber.sh
    cd "$CN_DIR"
fi

# 3. Add host route (ignore if it already exists)
sudo ip route add "$HOST_ROUTE_SUBNET" via "$HOST_ROUTE_GW" dev phine-net 2>/dev/null || true

# 4. Create ros_gz_net bridge
docker network create \
    --driver bridge \
    --subnet=192.168.73.128/26 \
    --opt com.docker.network.bridge.name="ros_gz_net" \
    ros_gz_net 2>/dev/null || true

# 5. Start DDS discovery server
cd "$DEMO_DIR"
docker compose -f oai/docker-compose-ue.yml up dds_discovery_server -d

# 6. Launch Gazebo simulation
export IGN_IP=192.168.73.129
ros2 launch ign_turtlebot gazebo_launch.launch.py &

# 7. Start RCC
cd "$DEMO_DIR"
xhost +local:docker
docker compose -f oai/docker-compose-ue.yml up rcc -d

echo "=== RoboSim5G native demo launched with $CORE_NETWORK ==="
