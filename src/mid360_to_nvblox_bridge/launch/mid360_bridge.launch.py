#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Launch file for Mid360 driver + nvblox bridge node (GPU)"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
        default_value='/mid360_bridge/output/pointcloud_structured',
        description='Output structured pointcloud topic for nvblox'
    )

    output_depth_image_topic_arg = DeclareLaunchArgument(
        'output_depth_image_topic',
        default_value='/mid360_bridge/output/depth_image',
        description='Output lidar depth image topic for nvblox'
    )

    # Livox Mid360 driver
    livox_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            '/workspaces/livox/livox_ros2_ws/install/livox_ros_driver2/share/livox_ros_driver2/launch_ROS2/msg_MID360_pointcloud2_launch.py'
        )
    )

    # Static TF: camera_link -> lidar -> livox_frame
    static_tf_camera_to_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['-0.05', '-0.03', '0.055',
                   '0', '0.263', '0', '0.965',
                   'camera_link', 'lidar']
    )
    static_tf_lidar_to_livox_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0',
                   '0', '0', '0', '1.0',
                   'lidar', 'livox_frame']
    )

    # GPU bridge node (5s delay to wait for driver to publish)
    bridge_node = TimerAction(
        period=5.0,
        actions=[Node(
            package='mid360_to_nvblox_bridge',
            executable='mid360_bridge_node_gpu_exe',
            name='mid360_bridge',
            output='screen',
            parameters=[LaunchConfiguration('config_file')],
            remappings=[
                ('~/input/pointcloud', LaunchConfiguration('input_pointcloud_topic')),
                ('~/output/structured_pointcloud', LaunchConfiguration('output_pointcloud_topic')),
                ('~/output/depth_image', LaunchConfiguration('output_depth_image_topic')),
            ]
        )]
    )

    return LaunchDescription([
        config_file_arg,
        input_topic_arg,
        output_topic_arg,
        output_depth_image_topic_arg,
        livox_driver,
        static_tf_camera_to_lidar,
        static_tf_lidar_to_livox_frame,
        bridge_node,
    ])
