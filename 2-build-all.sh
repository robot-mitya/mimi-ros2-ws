#!/bin/bash
set -e

echo "[INFO] Activating environment..."
source /opt/ros/kilted/setup.bash
source "$(dirname "$0")/.venv/bin/activate"

echo "[INFO] Building all packages..."
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -G Ninja
colcon build --packages-select mimi_ble --cmake-args -DPYTHON_EXECUTABLE=$(which python)

#source "$(dirname "$0")/install/setup.bash"
#export PYTHONPATH=$PYTHONPATH:$(find "$(dirname "$0")/install" -type d -name site-packages | paste -sd:)
