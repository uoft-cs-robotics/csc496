#!/usr/bin/env bash
cd slicer_ros2_module
source /opt/ros/humble/setup.bash
colcon build --cmake-args \
  -DSlicer_DIR:PATH=/home/csc379/slicer/Slicer-SuperBuild-Debug/Slicer-build \
  -DCMAKE_BUILD_TYPE=Release
