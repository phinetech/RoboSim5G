#!/bin/bash
# Check OAI AMF logs for gNB and UE registration
# Usage: ./check_registration.sh [timeout_seconds]

set -e

TIMEOUT=${1:-600}
INTERVAL=15
ELAPSED=0

GNB_CONTAINER_UP=false
UE_CONTAINER_UP=false
GNB_REGISTERED=false
UE_REGISTERED=false

echo "============================================"
echo "Integration Test: gNB and UE Registration"
echo "Timeout: ${TIMEOUT}s"
echo "============================================"

# Phase 1: Wait for gNB and UE containers to be started by the plugins
echo ""
echo "Phase 1: Waiting for gNB and UE containers to appear..."
echo "(The UE plugin builds a Docker image first, this can take several minutes)"
echo ""

while [ $ELAPSED -lt $TIMEOUT ]; do
    # Check if gNB container exists
    if [ "$GNB_CONTAINER_UP" = false ]; then
        if docker ps --format '{{.Names}}' 2>/dev/null | grep -q "oai-gNB"; then
            echo "[OK] gNB container is running (elapsed: ${ELAPSED}s)"
            GNB_CONTAINER_UP=true
        fi
    fi

    # Check if UE container exists (ue_turtlebot from the SDF config)
    if [ "$UE_CONTAINER_UP" = false ]; then
        if docker ps --format '{{.Names}}' 2>/dev/null | grep -qE "(ue_turtlebot|robot_1)"; then
            echo "[OK] UE container is running (elapsed: ${ELAPSED}s)"
            UE_CONTAINER_UP=true
        fi
    fi

    # Both containers running, move to phase 2
    if [ "$GNB_CONTAINER_UP" = true ] && [ "$UE_CONTAINER_UP" = true ]; then
        echo ""
        echo "Phase 2: Checking AMF logs for registration..."
        echo ""
        break
    fi

    # Show what's happening
    if [ $((ELAPSED % 60)) -eq 0 ] && [ $ELAPSED -gt 0 ]; then
        echo "  Still waiting... (elapsed: ${ELAPSED}s)"
        echo "  Running containers:"
        docker ps --format "    {{.Names}} ({{.Status}})" 2>/dev/null || true
    else
        echo "  Waiting... gNB_container=${GNB_CONTAINER_UP} UE_container=${UE_CONTAINER_UP} (elapsed: ${ELAPSED}s)"
    fi

    sleep $INTERVAL
    ELAPSED=$((ELAPSED + INTERVAL))
done

if [ "$GNB_CONTAINER_UP" = false ] || [ "$UE_CONTAINER_UP" = false ]; then
    echo "============================================"
    echo "[FAILURE] Timeout waiting for containers to start"
    echo "  gNB container running: ${GNB_CONTAINER_UP}"
    echo "  UE container running:  ${UE_CONTAINER_UP}"
    echo "============================================"
    echo ""
    echo "--- Running containers ---"
    docker ps --format "table {{.Names}}\t{{.Status}}\t{{.Image}}" 2>/dev/null || true
    echo ""
    echo "--- Simulation container logs (last 50 lines) ---"
    docker logs simulation 2>&1 | tail -50 || true
    exit 1
fi

# Phase 2: Check AMF logs for registration
while [ $ELAPSED -lt $TIMEOUT ]; do
    AMF_LOGS=$(docker logs oai-amf 2>&1 || echo "")

    # Check for gNB registration
    if echo "$AMF_LOGS" | grep -qiE "(gNB.*associated|NG_SETUP.*SUCCESS|Connected.*gNB|gNodeB.*context)"; then
        if [ "$GNB_REGISTERED" = false ]; then
            echo "[PASS] gNB registered with AMF (elapsed: ${ELAPSED}s)"
            GNB_REGISTERED=true
        fi
    fi

    # Check for UE registration
    if echo "$AMF_LOGS" | grep -qiE "(registration.*accept|5GMM-REGISTERED|UE.*registered|InitialUEMessage|REGISTRATION_COMPLETE)"; then
        if [ "$UE_REGISTERED" = false ]; then
            echo "[PASS] UE registered with AMF (elapsed: ${ELAPSED}s)"
            UE_REGISTERED=true
        fi
    fi

    # Both registered - success
    if [ "$GNB_REGISTERED" = true ] && [ "$UE_REGISTERED" = true ]; then
        echo ""
        echo "============================================"
        echo "[SUCCESS] Both gNB and UE are registered with the AMF"
        echo "============================================"
        echo ""
        echo "--- Relevant AMF log lines ---"
        docker logs oai-amf 2>&1 | grep -iE "(gNB|UE|registration|associated|connected|NGAP)" | tail -20 || true
        exit 0
    fi

    echo "  Checking AMF... gNB=${GNB_REGISTERED} UE=${UE_REGISTERED} (elapsed: ${ELAPSED}s)"
    sleep $INTERVAL
    ELAPSED=$((ELAPSED + INTERVAL))
done

echo ""
echo "============================================"
echo "[FAILURE] Timeout after ${TIMEOUT}s"
echo "  gNB registered: ${GNB_REGISTERED}"
echo "  UE registered:  ${UE_REGISTERED}"
echo "============================================"
echo ""
echo "--- AMF logs (last 80 lines) ---"
docker logs oai-amf 2>&1 | tail -80 || true
echo ""
echo "--- gNB container logs ---"
docker logs oai-gNB1 2>&1 | tail -30 || true
echo ""
echo "--- UE container logs ---"
docker logs ue_turtlebot 2>&1 | tail -30 || true
echo ""
echo "--- Running containers ---"
docker ps --format "table {{.Names}}\t{{.Status}}\t{{.Image}}" 2>/dev/null || true
exit 1
