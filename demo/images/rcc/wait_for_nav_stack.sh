#!/bin/bash

echo "Starting conservative readiness check..."

# 1. Wait for Lifecycle Manager
until ros2 service call /lifecycle_manager_navigation/is_active std_srvs/srv/Trigger {} 2>/dev/null | grep -q "success=True"; do
  echo "Waiting for Nav2 Lifecycle Manager..."
  sleep 2
done

# 2. Wait for Global Costmap to start streaming
# We use 'ros2 topic hz' with a 1-message limit to see if it's alive
echo "Nav2 Active. Waiting for Global Costmap to publish..."
until ros2 topic hz /global_costmap/costmap 2>/dev/null | grep -q "average rate"; do
  echo "Costmap not streaming yet..."
  sleep 2
done

# 3. Wait for Local Costmap to start streaming
# The controller server needs this to be ready
echo "Global Costmap detected. Waiting for Local Costmap to publish..."
until ros2 topic hz /local_costmap/costmap 2>/dev/null | grep -q "average rate"; do
  echo "Local costmap not streaming yet..."
  sleep 2
done

# 4. Check if Transform Tree (TF) is valid
# This ensures AMCL/Localization has actually linked the robot to the map.
echo "Local Costmap detected. Waiting for Map -> Base Link transform..."
until ros2 run tf2_ros tf2_echo map base_link 2>/dev/null | grep -q "Rotation"; do
  echo "Localization not settled..."
  sleep 2
done

# 5. Wait for the Navigate to Pose action server to be available
# This ensures the behavior tree and controller server are ready to accept goals
echo "Localization ready. Waiting for Navigate to Pose action server..."
until ros2 action list 2>/dev/null | grep -q "/navigate_to_pose"; do
  echo "Navigate to Pose action server not available yet..."
  sleep 2
done

# 6. Give controller server a moment to fully initialize after action server appears
# This allows the controller to process initial costmap data and warm up
echo "Action server available. Allowing controller server to warm up..."
sleep 3

echo "ROBOT IS FULLY READY. Starting RViz..."
exec "$@"