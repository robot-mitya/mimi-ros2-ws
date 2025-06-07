#!/bin/bash
set -e

echo "[INFO] Activating ROS 2 environment..."
source /opt/ros/kilted/setup.bash

echo "[INFO] Deleting 'build', 'install', and 'log' directories..."
rm -rf build install log
