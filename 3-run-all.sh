#!/bin/bash
set -e

echo "[INFO] Activating ROS 2 environment..."
source /opt/ros/kilted/setup.bash

echo "[INFO] Activating workspace environment..."
source install/setup.bash

echo "[INFO] Running all packages..."
ros2 launch mimi_bringup launch.py
