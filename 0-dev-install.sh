#!/bin/bash
set -e

echo "[INFO] Activating ROS 2 environment..."
source /opt/ros/kilted/setup.bash

echo "[INFO] Running rosdep to install system dependencies..."
rosdep update
rosdep install --from-paths src --ignore-src -y --rosdistro=kilted

echo "[INFO] Setting up shared virtual environment in workspace root..."
python3 -m venv .venv
source .venv/bin/activate

echo "[INFO] Upgrading pip and setuptools..."
pip install -U pip setuptools
pip install empy catkin_pkg lark-parser pyyaml numpy

echo "[INFO] Installing all Python packages in editable mode..."
for pkg in src/*/; do
  if [ -f "$pkg/setup.py" ] || [ -f "$pkg/pyproject.toml" ]; then
    echo "  ↪ $pkg"
    pip install -e "$pkg"
  fi
done

echo "[SUCCESS] Development environment is ready. Don't forget to run:"
echo "  source .venv/bin/activate"
