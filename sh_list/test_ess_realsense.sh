#!/bin/bash

source /opt/ros/humble/setup.bash

echo "Test Launch Ess Depth ..."
ros2 launch isaac_ros_examples isaac_ros_examples.launch.py launch_fragments:=realsense_stereo_rect,ess_disparity \
   engine_file_path:=${ISAAC_ROS_WS}/isaac_ros_assets/models/dnn_stereo_disparity/dnn_stereo_disparity_v4.1.0_onnx/ess.engine \
   threshold:=0.4 realsense_config_file:=$(ros2 pkg prefix isaac_ros_ess --share)/config/realsense.yaml
