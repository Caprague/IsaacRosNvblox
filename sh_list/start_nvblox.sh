#!/bin/bash

SESSION_NAME="nvblox_safety"

# 创建新会话，在左 pane 执行

tmux new-session -d -s "$SESSION_NAME" -n "main"

# 左 pane: source 后即刻启动 nvblox launch
tmux send-keys -t "$SESSION_NAME:main.0" \
  "source /opt/ros/humble/setup.bash && source /workspaces/isaac_ros-dev/install/setup.bash && ros2 launch nvblox_examples_bringup realsense_example_usr.launch.py" \
  C-m

# 水平分割，创建右 pane
tmux split-window -h -t "$SESSION_NAME:main.0"

# 右 pane: source 后等待 16s，再启动 safety_guardian
tmux send-keys -t "$SESSION_NAME:main.1" \
  "source /opt/ros/humble/setup.bash && source /workspaces/isaac_ros-dev/install/setup.bash && sleep 16 && ros2 run safety_guardian safety_guardian_node" \
  C-m

# 附加到会话
tmux attach-session -t "$SESSION_NAME"
