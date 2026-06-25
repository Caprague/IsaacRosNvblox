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

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, LogInfo
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    """Launch file for the Safety Guardian node with auto-restart (always on mode)."""

    # 获取包路径
    package_dir = get_package_share_directory('safety_guardian')

    # 获取参数文件路径
    params_file = os.path.join(package_dir, 'config', 'safety_guardian_params.yaml')

    # 创建安全保护节点
    safety_guardian_node = Node(
        package='safety_guardian',
        executable='safety_guardian_node',
        name='safety_guardian_node',
        output='screen',
        parameters=[params_file],
        arguments=['--ros-args', '--log-level', 'info'],
        # 启用 respawn 功能（类似 ROS1 的 always_on）
        respawn=True,
        respawn_delay=1.0  # 重启延迟 1 秒
    )

    # 添加事件处理器，当节点退出时记录日志
    exit_event_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=safety_guardian_node,
            on_exit=[
                LogInfo(msg='Safety Guardian node exited. Auto-restarting in 2 seconds...')
            ]
        )
    )

    return LaunchDescription([
        safety_guardian_node,
        exit_event_handler,
        LogInfo(msg='Safety Guardian node started in always-on mode (auto-restart enabled)')
    ])