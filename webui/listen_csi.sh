#!/bin/bash
# Helper script to listen to CSI data from host

# Set ROS2 environment to match container
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

# Source ROS2 and wifi_msgs
source /opt/ros/humble/setup.bash
source /opt/ros2-csi-msgs/install/setup.bash

# Restart daemon with correct environment
ros2 daemon stop
ros2 daemon start
sleep 1

echo "ROS2 Environment configured. Available topics:"
ros2 topic list

echo ""
echo "Listening to /csi_data..."
ros2 topic echo /csi_data
