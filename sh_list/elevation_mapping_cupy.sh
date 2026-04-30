#!/bin/bash

source /opt/ros/humble/setup.bash
source /workspaces/isaac_ros-dev/install/setup.bash

echo "Realsense D435if - Elavation Mapping Cupy >>> Launching ... "
ros2 launch elevation_mapping_cupy elevation_mapping_cupy.launch.py

