#!/bin/bash
# Wait for free5gc core network to be fully loaded
# MongoDB is the primary dependency for the control plane NFs.

set -e

TIMEOUT=${1:-300}  # Default timeout: 5 minutes
INTERVAL=5

echo "Waiting for free5gc core network to be ready..."
echo "Checking mongodb container status..."

elapsed=0
while [ $elapsed -lt $TIMEOUT ]; do
    # 1. Check if MongoDB is running
    # Note: Your mongo image doesn't have a native healthcheck defined in YAML, 
    # so we check if the container state is 'running'.
    db_status=$(docker inspect --format='{{.State.Status}}' mongodb 2>/dev/null || echo "not_found")
    
    if [ "$db_status" = "running" ]; then
        echo "✓ MongoDB container is running"
        
        # 2. Verify critical free5gc services are running
        # These names match your 'container_name' fields in the docker-compose
        services=("nrf" "amf" "smf" "upf" "webui")
        all_running=true
        
        for service in "${services[@]}"; do
            if ! docker ps --format '{{.Names}}' | grep -q "^${service}$"; then
                all_running=false
                echo "  Waiting for $service to start..."
                break
            fi
        done
        
        if [ "$all_running" = true ]; then
            # Check if NRF is ready by examining its logs for startup message
            # Look for "SBI server started" which indicates NRF is ready
            if docker logs nrf 2>&1 | grep -q "SBI server started"; then
                echo "✓ NRF service is ready (SBI server started)"
                echo "free5gc core network is ready!"
                exit 0
            else
                echo "  All containers running, waiting for NRF to start SBI server (elapsed: ${elapsed}s)..."
            fi
        fi
    else
        echo "  MongoDB status: $db_status (elapsed: ${elapsed}s)"
    fi
    
    sleep $INTERVAL
    elapsed=$((elapsed + INTERVAL))
done

echo "ERROR: Timeout waiting for free5gc core network to be ready after ${TIMEOUT}s"
exit 1