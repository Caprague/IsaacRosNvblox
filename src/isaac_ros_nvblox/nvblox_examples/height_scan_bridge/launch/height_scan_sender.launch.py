# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        # Height scan sender args
        DeclareLaunchArgument(
            'height_scan_target_ip',
            default_value='192.168.123.18',
            description='Remote host IP for height scan UDP sending'),
        DeclareLaunchArgument(
            'height_scan_port',
            default_value='9870',
            description='UDP port for height scan sending'),
        DeclareLaunchArgument(
            'height_scan_print_stats',
            default_value='true',
            description='Height scan: print stats (true/false)'),
        # Safety guardian sender args
        DeclareLaunchArgument(
            'safety_guardian_target_ip',
            default_value='192.168.123.18',
            description='Remote host IP for safety guardian UDP sending'),
        DeclareLaunchArgument(
            'safety_guardian_port',
            default_value='9871',
            description='UDP port for safety guardian sending'),
        DeclareLaunchArgument(
            'safety_guardian_print_stats',
            default_value='true',
            description='Safety guardian: print stats (true/false)'),
        # Height scan sender node
        Node(
            package='height_scan_bridge',
            executable='udp_height_scan_sender',
            name='height_scan_udp_sender',
            output='screen',
            parameters=[{
                'target_ip': LaunchConfiguration('height_scan_target_ip'),
                'port': LaunchConfiguration('height_scan_port'),
                'print_stats': LaunchConfiguration('height_scan_print_stats'),
            }],
        ),
        # Safety guardian sender node
        Node(
            package='height_scan_bridge',
            executable='udp_safety_guardian_sender',
            name='safety_guardian_udp_sender',
            output='screen',
            parameters=[{
                'target_ip': LaunchConfiguration('safety_guardian_target_ip'),
                'port': LaunchConfiguration('safety_guardian_port'),
                'print_stats': LaunchConfiguration('safety_guardian_print_stats'),
            }],
        ),
    ])
