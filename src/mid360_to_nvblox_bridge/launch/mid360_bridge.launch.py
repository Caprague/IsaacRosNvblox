#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Launch file for Mid360 to nvblox bridge node (GPU)"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('mid360_to_nvblox_bridge'),
            'config',
            'mid360_bridge_gpu.yaml'
        ]),
        description='Path to configuration file'
    )

    input_topic_arg = DeclareLaunchArgument(
        'input_pointcloud_topic',
        default_value='/livox/lidar',
        description='Input Mid360 pointcloud topic'
    )

    output_topic_arg = DeclareLaunchArgument(
        'output_pointcloud_topic',
        default_value='/lidar/pointcloud_structured',
        description='Output structured pointcloud topic for nvblox'
    )

    # GPU bridge node
    bridge_node = Node(
        package='mid360_to_nvblox_bridge',
        executable='mid360_bridge_node_gpu_exe',
        name='mid360_bridge',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
        remappings=[
            ('~/input/pointcloud', LaunchConfiguration('input_pointcloud_topic')),
            ('~/output/structured_pointcloud', LaunchConfiguration('output_pointcloud_topic')),
        ]
    )

    return LaunchDescription([
        config_file_arg,
        input_topic_arg,
        output_topic_arg,
        bridge_node,
    ])
