#!/bin/bash
set -e

# 1. Setup ROS environment
source /opt/ros/kilted/setup.bash

# 2. Workspace root
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_JSON="$WS_DIR/build/compile_commands.json"
ROOT_JSON="$WS_DIR/compile_commands.json"

# 3. Build if compile_commands.json is missing
if [ ! -f "$BUILD_JSON" ]; then
    echo "[INFO] compile_commands.json not found, running colcon build..."
    colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -G Ninja
else
    echo "[INFO] Found: $BUILD_JSON"
fi

# 4. Create or update symlink in root
if [ ! -L "$ROOT_JSON" ]; then
    echo "[INFO] Creating symlink: compile_commands.json → $BUILD_JSON"
    ln -sf "build/compile_commands.json" "$ROOT_JSON"
else
    echo "[INFO] Symlink already exists: $ROOT_JSON"
fi

# 5. Launch CLion
echo "[INFO] Launching CLion..."
clion "$ROOT_JSON"
