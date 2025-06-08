#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR=$SCRIPT_DIR
WS_DIR="$(cd "$(dirname "${PKG_DIR}/../../..")" && pwd)"

source /opt/ros/kilted/setup.bash

cd "$WS_DIR"
colcon build --packages-select mimi_common --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -G Ninja
