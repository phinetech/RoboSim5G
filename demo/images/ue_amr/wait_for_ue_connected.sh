#!/bin/bash

# Wait until oaitun_ue1 is up and has an IP address
INTERFACE="oaitun_ue1"
TIMEOUT=120  # seconds, set to 0 for infinite wait
INTERVAL=2   # seconds between checks
ELAPSED=0

echo "[INFO] Waiting for UE to connect (interface $INTERFACE with IP)..."

while true; do
    if ip addr show "$INTERFACE" 2>/dev/null | grep -q "inet "; then
        echo "[SUCCESS] UE is connected. Interface $INTERFACE is up with IP."
        exit 0
    fi

    sleep $INTERVAL
    ((ELAPSED+=INTERVAL))

    if [ $TIMEOUT -gt 0 ] && [ $ELAPSED -ge $TIMEOUT ]; then
        echo "[ERROR] Timeout reached: UE did not connect within $TIMEOUT seconds."
        exit 1
    fi
done
