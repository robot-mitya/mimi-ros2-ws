#!/bin/bash
set -e

echo "[INFO] Activating ROS 2 environment..."
source /opt/ros/kilted/setup.bash

echo "[INFO] Running rosdep to install system dependencies..."
rosdep update
rosdep install --from-paths src --ignore-src -y --rosdistro=kilted

# Setup all known Python virtual environments
for pkg in src/*/; do
  if [ -f "$pkg/pyproject.toml" ] || [ -f "$pkg/setup.py" ]; then
    echo "[INFO] Setting up Python package: $pkg"

    cd "$pkg"
    if [ ! -d ".venv" ]; then
      echo "[INFO] Creating venv in $pkg"
      python3 -m venv .venv
    fi

    source .venv/bin/activate
    pip install -U pip setuptools
    pip install -e .
    deactivate
    cd - > /dev/null
  fi
done

echo "[SUCCESS] Development environment is ready."
