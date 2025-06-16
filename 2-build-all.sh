#!/bin/bash
set -e

echo "[INFO] Activating environment..."
source /opt/ros/kilted/setup.bash

echo "[INFO] Building all packages..."
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -G Ninja
