#!/bin/bash

source /opt/ros/humble/setup.bash
source /workspaces/isaac_ros-dev/install/setup.bash

echo "Realsense D435if - Terrain Sampling >>> Launching ... "
ros2 launch terrain_sampling_node terrain_sampling.launch.py

