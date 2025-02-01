#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DSlicer_DIR:PATH=/home/csc379/slicer/Slicer-SuperBuild-Debug/Slicer-build -DCMAKE_BUILD_TYPE=Release --parallel-workers 4
