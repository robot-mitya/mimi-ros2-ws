#!/bin/bash
source /opt/ros/kilted/setup.bash
source ../../install/setup.bash

export PYTHONPATH=$PYTHONPATH:$(find ../../install -type d -name site-packages | paste -sd:)
exec "pycharm-community"
