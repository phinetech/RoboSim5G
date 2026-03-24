#!/bin/bash
# Wait for Open5GS core network to be fully loaded
# The database (MongoDB) is the longest thing to load and is a dependency for other services

set -e

TIMEOUT=${1:-300}  # Default timeout: 5 minutes
INTERVAL=5

echo "Waiting for Open5GS core network to be ready..."
echo "Checking MongoDB container status..."

elapsed=0
while [ $elapsed -lt $TIMEOUT ]; do
    # Check if MongoDB container is running
    db_running=$(docker ps --format '{{.Names}}' | grep "^db$" || echo "not_found")
    
    if [ "$db_running" = "db" ]; then
        # Verify MongoDB is accepting connections
        mongo_ready=$(docker exec db mongosh --eval "db.adminCommand('ping')" --quiet 2>/dev/null | grep -q "ok" && echo "ready" || echo "not_ready")
        
        if [ "$mongo_ready" = "ready" ]; then
            echo "✓ MongoDB is ready and accepting connections"
            
            # Additional check: verify critical services are running
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
                echo "✓ All critical Open5GS services are running"
                echo "Open5GS core network is ready!"
                exit 0
            fi
        else
            echo "  MongoDB is starting up... (elapsed: ${elapsed}s)"
        fi
    else
        echo "  MongoDB container not found (elapsed: ${elapsed}s)"
    fi
    
    sleep $INTERVAL
    elapsed=$((elapsed + INTERVAL))
done

echo "ERROR: Timeout waiting for Open5GS core network to be ready after ${TIMEOUT}s"
exit 1
