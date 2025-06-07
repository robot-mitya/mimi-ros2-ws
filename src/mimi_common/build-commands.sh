#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR=$SCRIPT_DIR
BUILD_DIR="$PKG_DIR/build"

source /opt/ros/kilted/setup.bash

mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"
cmake "$PKG_DIR" -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -G Ninja -DCMAKE_INSTALL_PREFIX="$INSTALL_DIR"
cmake --build . -- -j8 -l8
