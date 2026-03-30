#!/bin/bash
# Wait for free5GC core network to be fully loaded
# MongoDB is the first service to come up and is a dependency for other services

set -e

TIMEOUT=${1:-300}  # Default timeout: 5 minutes
INTERVAL=5

echo "Waiting for free5GC core network to be ready..."
echo "Checking mongodb container status..."

elapsed=0
while [ $elapsed -lt $TIMEOUT ]; do
    # Check if mongodb container is running
    mongo_status=$(docker inspect --format='{{.State.Running}}' mongodb 2>/dev/null || echo "false")

    if [ "$mongo_status" = "true" ]; then
        echo "MongoDB container is running"

        # Check critical free5GC services
        services=("nrf" "amf" "smf" "upf")
        all_running=true

        for service in "${services[@]}"; do
            if ! docker ps --format '{{.Names}}' | grep -q "^${service}$"; then
                all_running=false
                echo "  Waiting for $service to start..."
                break
            fi
        done

        if [ "$all_running" = true ]; then
            echo "All critical free5GC services are running!"
            # Give services a moment to fully initialize
            sleep 10
            echo "free5GC core network is ready."
            exit 0
        fi
    fi

    sleep $INTERVAL
    elapsed=$((elapsed + INTERVAL))
    echo "  Elapsed: ${elapsed}s / ${TIMEOUT}s"
done

echo "ERROR: Timeout waiting for free5GC core network to be ready."
exit 1
