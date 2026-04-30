#!/bin/bash

SESSION_NAME="Elevation Perception & Mapping"
SCRIPT_A="/workspaces/isaac_ros-dev/sh_list/ess_pointcloud_realsense.sh"
SCRIPT_B="/workspaces/isaac_ros-dev/sh_list/elevation_mapping_cupy.sh"
SCRIPT_C="/workspaces/isaac_ros-dev/sh_list/terrain_sapmle.sh"

# 创建新会话
tmux new-session -d -s "$SESSION_NAME" -n "main"

# 运行脚本A (左上，pane 0)
tmux send-keys -t "$SESSION_NAME:main.0" "$SCRIPT_A" C-m

# 垂直分割，创建 pane 1（下方）
tmux split-window -v -t "$SESSION_NAME:main.0"
# 对上方 pane (0) 水平分割，创建 pane 2（右上）
tmux split-window -h -t "$SESSION_NAME:main.0"
# 对下方 pane (1) 水平分割，创建 pane 3（右下）
tmux split-window -v -t "$SESSION_NAME:main.1"

# 运行脚本B（左下，pane 1）
tmux send-keys -t "$SESSION_NAME:main.1" "sleep 10.0 && $SCRIPT_B" C-m

# 运行脚本C（右上，pane 2)
tmux send-keys -t "$SESSION_NAME:main.2" "sleep 20.0 && $SCRIPT_C" C-m

# pane 3
tmux send-keys -t "$SESSION_NAME:main.3" "bash && source /opt/ros/humble/setup.bash" C-m

# 附加到会话
tmux attach-session -t "$SESSION_NAME"

