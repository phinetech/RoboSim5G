#!/bin/bash
# Wait for OAI core network to be fully loaded
# The database (MySQL) is the longest thing to load and is a dependency for other services

set -e

TIMEOUT=${1:-300}  # Default timeout: 5 minutes
INTERVAL=5

echo "Waiting for OAI core network to be ready..."
echo "Checking mysql container health status..."

elapsed=0
while [ $elapsed -lt $TIMEOUT ]; do
    # Check if mysql container is healthy
    mysql_health=$(docker inspect --format='{{.State.Health.Status}}' mysql 2>/dev/null || echo "not_found")
    
    if [ "$mysql_health" = "healthy" ]; then
        echo "✓ MySQL container is healthy"
        
        # Additional check: verify critical services are running
        services=("oai-nrf" "oai-amf" "oai-smf" "oai-upf")
        all_running=true
        
        for service in "${services[@]}"; do
            if ! docker ps --format '{{.Names}}' | grep -q "^${service}$"; then
                all_running=false
                echo "  Waiting for $service to start..."
                break
            fi
        done
        
        if [ "$all_running" = true ]; then
            echo "✓ All critical OAI services are running"
            echo "OAI core network is ready!"
            exit 0
        fi
    else
        echo "  MySQL status: $mysql_health (elapsed: ${elapsed}s)"
    fi
    
    sleep $INTERVAL
    elapsed=$((elapsed + INTERVAL))
done

echo "ERROR: Timeout waiting for OAI core network to be ready after ${TIMEOUT}s"
exit 1
