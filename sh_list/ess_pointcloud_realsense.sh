#!/bin/bash

source /opt/ros/humble/setup.bash
source /workspaces/isaac_ros-dev/install/setup.bash

echo "Realsense D435if - Isaac Ros - VSlam & Ess & Pointcloud >>> Launching ... "
ros2 launch isaac_ros_ess realsense_ess_pointcloud.launch.py

