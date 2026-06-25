#!/bin/bash

SESSION_NAME="nvblox_lvi"
SOURCE_CMD="source /opt/ros/humble/setup.bash && source /workspaces/livox/livox_ros2_ws/install/setup.bash && source /workspaces/isaac_ros-dev/install/setup.bash"

# 创建新会话
tmux new-session -d -s "$SESSION_NAME" -n "main"

# 水平分割: 左(0) | 右(1)
tmux split-window -h -t "$SESSION_NAME:main.0"

# 垂直分割左列: 左上(0) | 左下(1)
tmux split-window -v -t "$SESSION_NAME:main.0"

# 垂直分割右列: 右上(2) | 右下(3)
tmux split-window -v -t "$SESSION_NAME:main.2"

# 1号屏(左上): source 后启动 mid360 bridge
tmux send-keys -t "$SESSION_NAME:main.0" \
  "$SOURCE_CMD && ros2 launch mid360_to_nvblox_bridge mid360_bridge.launch.py" \
  C-m

# 2号屏(左下): 延时 10s 后启动 nvblox
tmux send-keys -t "$SESSION_NAME:main.1" \
  "$SOURCE_CMD && sleep 10 && ros2 launch nvblox_examples_bringup realsense_example_usr.launch.py" \
  C-m

# 3号屏(右上): 延时 26s 后启动 safety_guardian
tmux send-keys -t "$SESSION_NAME:main.2" \
  "$SOURCE_CMD && sleep 26 && ros2 launch safety_guardian safety_guardian_always_on.launch.py" \
  C-m

# 4号屏(右下): source 后留空
tmux send-keys -t "$SESSION_NAME:main.3" \
  "$SOURCE_CMD" \
  C-m

# 附加到会话
tmux attach-session -t "$SESSION_NAME"
