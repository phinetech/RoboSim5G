#!/bin/bash
# Unified kill script for RoboSim5G containerized demo
# Usage: CORE_NETWORK=oai|free5gc|open5gs ./kill.sh

CORE_NETWORK=${CORE_NETWORK:-oai}
REPO_ROOT=$(cd "$(dirname "$0")/.." && pwd)
DEMO_DIR=$(cd "$(dirname "$0")" && pwd)
CN_DIR="$REPO_ROOT/core_network_setup/$CORE_NETWORK"
export NAME_ROBOT_1=${NAME_ROBOT_1:-"ue_turtlebot"}

echo "=== Stopping RoboSim5G containerized demo ($CORE_NETWORK) ==="

# Stop UE services (simulation, rcc, dds)
cd "$DEMO_DIR"
docker compose -f oai/docker-compose-ue.yml down 

# Stop gNB containers
docker ps -q --filter ancestor=oaisoftwarealliance/oai-gnb:2026.w04 | xargs -r -I {} docker stop {}
docker ps -a -q --filter ancestor=oaisoftwarealliance/oai-gnb:2026.w04 | xargs -r -I {} docker rm {}

# Stop core network
cd "$CN_DIR"
docker compose down 2>/dev/null || true

# Remove networks
docker network rm ros_gz_net 2>/dev/null || true

# Revoke X11 access
xhost -local:docker 2>/dev/null || true

echo "=== RoboSim5G containerized demo stopped ==="
