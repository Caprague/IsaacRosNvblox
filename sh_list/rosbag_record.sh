#!/bin/bash

# 设置要录制的话题（请根据需要修改）
TOPICS=(
    "/visual_slam/tracking/slam_path"
    "/visual_slam/tracking/vo_pose"
    "/terrain_sampling_node/height_diff"
    "/terrain_sampling_node/terrain_sampling"
    "/tf"
    "/tf_static"
    "/elevation_mapping/elevation_map_filter"
)

# 生成带时间戳的文件夹名
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
BAG_PATH="/workspaces/isaac_ros-dev/rosbags/rosbag2_${TIMESTAMP}"

# 构建 ros2 bag record 命令
CMD="ros2 bag record -o \"${BAG_PATH}\""

for topic in "${TOPICS[@]}"; do
    CMD+=" ${topic}"
done

echo "Recording topics:"
printf ' - %s\n' "${TOPICS[@]}"
echo "Output bag path: ${BAG_PATH}"
echo "Command: ${CMD}"
echo "Press Ctrl+C to stop recording."

# 执行录制命令（前台运行，便于用 Ctrl+C 停止）
eval ${CMD}

