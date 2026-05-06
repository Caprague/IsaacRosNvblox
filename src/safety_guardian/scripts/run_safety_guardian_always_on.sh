#!/bin/bash
# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

# Safety Guardian Always-On Script
# 类似于 ROS1 的 respawn 功能，节点崩溃后自动重启

# 加载ROS环境
source /opt/ros/humble/setup.bash

# 加载工作区环境
WORKSPACE_DIR="/home/nvidia/IsaacRos/isaac_ros-dev"
if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then
    source "$WORKSPACE_DIR/install/setup.bash"
fi

echo "========================================"
echo "Safety Guardian - Always On Mode"
echo "========================================"
echo "Auto-restart enabled"
echo "Press Ctrl+C to stop"
echo "========================================"
echo ""

# 启动参数
LAUNCH_FILE="safety_guardian safety_guardian.launch.py"
RESTART_DELAY=2  # 重启延迟（秒）
MAX_RESTARTS=0   # 0 表示无限重启

restart_count=0

while true; do
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] Starting Safety Guardian node..."
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] Restart count: $restart_count"

    # 启动节点
    ros2 launch $LAUNCH_FILE &
    NODE_PID=$!

    # 等待节点退出
    wait $NODE_PID
    EXIT_CODE=$?

    echo "[$(date '+%Y-%m-%d %H:%M:%S')] Safety Guardian node exited with code: $EXIT_CODE"

    # 检查是否达到最大重启次数
    if [ $MAX_RESTARTS -gt 0 ] && [ $restart_count -ge $MAX_RESTARTS ]; then
        echo "[$(date '+%Y-%m-%d %H:%M:%S')] Maximum restart count reached. Stopping."
        break
    fi

    # 增加重启计数
    restart_count=$((restart_count + 1))

    # 等待一段时间后重启
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] Waiting ${RESTART_DELAY} seconds before restart..."
    sleep $RESTART_DELAY

    echo "[$(date '+%Y-%m-%d %H:%M:%S')] Restarting..."
    echo ""
done

echo "[$(date '+%Y-%m-%d %H:%M:%S')] Safety Guardian always-on script stopped."