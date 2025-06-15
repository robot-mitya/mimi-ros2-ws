#!/bin/bash
set -e

echo "[INFO] Activating ROS 2 environment..."
source /opt/ros/kilted/setup.bash

echo "[INFO] Activating workspace environment..."
source install/setup.bash
source "$(dirname "$0")/install/setup.bash"
source .venv/bin/activate
export PYTHONPATH=$PYTHONPATH:$(find "$(dirname "$0")/install" -type d -name site-packages | paste -sd:)

echo "[INFO] Running all packages..."
ros2 launch mimi_bringup launch.py
