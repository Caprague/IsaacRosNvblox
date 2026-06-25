#!/bin/bash

SESSION_NAME="nvblox_safety"
SOURCE_CMD="source /opt/ros/humble/setup.bash && source /workspaces/livox/livox_ros2_ws/install/setup.bash && source /workspaces/isaac_ros-dev/install/setup.bash"

# 创建新会话，在左 pane 执行

tmux new-session -d -s "$SESSION_NAME" -n "main"
tmux split-window -h -t "$SESSION_NAME:main.0"
tmux split-window -v -t "$SESSION_NAME:main.1"

# 左 pane: source 后即刻启动 nvblox launch
tmux send-keys -t "$SESSION_NAME:main.0" \
  "$SOURCE_CMD && ros2 launch nvblox_examples_bringup realsense_x2_usr.launch.py" \
  C-m

# 右上 pane: source 后等待 40s，再启动 safety_guardian
tmux send-keys -t "$SESSION_NAME:main.1" \
  "$SOURCE_CMD && sleep 40 && ros2 launch safety_guardian safety_guardian_always_on.launch.py" \
  C-m

# 右下 pane: source 后留空
tmux send-keys -t "$SESSION_NAME:main.2" \
  "$SOURCE_CMD" \
  C-m

# 附加到会话
tmux attach-session -t "$SESSION_NAME"
