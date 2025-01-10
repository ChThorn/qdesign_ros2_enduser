#!/bin/bash

# Source ROS2 environment
source /opt/ros/humble/setup.bash
source ~/qdesign_ros2/install/setup.bash

# Start camera node in background
ros2 run qdesign_ros2 realsense_camera_node &
CAMERA_PID=$!

# Wait a moment for camera node to initialize
sleep 2

# Start Qt viewer
ros2 run qdesign_ros2 qt_realsense_node

# When Qt viewer closes, kill camera node
kill $CAMERA_PID