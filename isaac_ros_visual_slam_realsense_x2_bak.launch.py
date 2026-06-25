# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2021-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

import launch
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def get_primary_realsense_parameters(camera_name):
    return {
        'camera_name': camera_name,
        'enable_infra1': True,
        'enable_infra2': True,
        'enable_color': True,
        'enable_depth': True,
        'depth_module.emitter_enabled': 0,
        'depth_module.profile': '640x360x90',
        'enable_gyro': True,
        'enable_accel': True,
        'gyro_fps': 200,
        'accel_fps': 200,
        'unite_imu_method': 2,
    }


def get_secondary_realsense_parameters(camera_name):
    return {
        'camera_name': camera_name,
        'enable_infra1': False,
        'enable_infra2': False,
        'enable_color': False,
        'enable_depth': True,
        'depth_module.emitter_enabled': 0,
        'depth_module.profile': '640x480x60',
        'enable_gyro': False,
        'enable_accel': False,
        'gyro_fps': 200,
        'accel_fps': 200,
        'unite_imu_method': 2,
    }


def create_realsense_node(camera_name, serial_number=None):
    if camera_name == 'camera0':
        parameters = get_primary_realsense_parameters(camera_name)
    else:
        parameters = get_secondary_realsense_parameters(camera_name)

    if serial_number:
        parameters['serial_no'] = serial_number

    return Node(
        name='camera',
        namespace=camera_name,
        package='realsense2_camera',
        executable='realsense2_camera_node',
        parameters=[parameters],
        output='screen',
    )


def launch_realsense_nodes(context, *args, **kwargs):
    num_cameras = int(LaunchConfiguration('num_cameras').perform(context))
    serial_numbers_arg = LaunchConfiguration('camera_serial_numbers').perform(context)
    serial_numbers = [serial.strip() for serial in serial_numbers_arg.split(',') if serial.strip()]

    if serial_numbers and len(serial_numbers) < num_cameras:
        raise RuntimeError(
            'camera_serial_numbers must be empty or contain at least num_cameras entries. '
            f'Got {len(serial_numbers)} serials for {num_cameras} cameras.')

    actions = []
    for idx in range(num_cameras):
        camera_name = f'camera{idx}'
        serial_number = serial_numbers[idx] if serial_numbers else None
        actions.append(
            TimerAction(
                period=idx * 30.0,
                actions=[create_realsense_node(camera_name, serial_number)],
            )
        )
    return actions


def generate_launch_description():
    """Launch file which brings up visual slam node configured for RealSense."""
    camera_serial_numbers_arg = DeclareLaunchArgument(
        'camera_serial_numbers',
        default_value='336222074023,242622071196',
        description='Comma-separated RealSense serial numbers. Empty means auto-select devices.',
    )
    num_cameras_arg = DeclareLaunchArgument(
        'num_cameras',
        default_value='2',
        description='Number of independent RealSense camera nodes to launch.',
    )

    imu_horizontal_align_node = Node(
        name='imu_horizontal_align_node',
        package='odom_horizontal_transform',
        executable='imu_horizontal_align',
        output='screen',
        parameters=[{
            'start_delay': 8.0,  # 延迟启动水平校正过程
            'imu_topic': '/camera0/imu',
            'use_static_tf_broadcaster': True,
            'enable_horizontal_pose_bridge': True,
            'vslam_pose_topic': '/visual_slam/tracking/vo_pose',
            'horizontal_pose_topic': '/visual_slam/tracking/pose_odom_horizontal_base',
            'horizontal_global_frame': 'odom_horizontal',
            'odom_frame': 'odom',
            'vslam_pose_frame': 'camera0_link',
            'base_frame': 'base_link',
        }]
    )

    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        parameters=[{
            'enable_image_denoising': True,
            'rectified_images': True,
            'enable_imu_fusion': True,
            'gyro_noise_density': 0.000244,
            'gyro_random_walk': 0.000019393,
            'accel_noise_density': 0.001862,
            'accel_random_walk': 0.003,
            'calibration_frequency': 200.0,
            'image_jitter_threshold_ms': 23.00,
            'base_frame': 'camera0_link',
            'imu_frame': 'camera0_gyro_optical_frame',
            'enable_slam_visualization': True,
            'enable_landmarks_view': True,
            'enable_observations_view': True,
            'camera_optical_frames': [
                'camera0_infra1_optical_frame',
                'camera0_infra2_optical_frame',
            ],
        }],
        remappings=[
            ('visual_slam/image_0', 'camera0/infra1/image_rect_raw'),
            ('visual_slam/camera_info_0', 'camera0/infra1/camera_info'),
            ('visual_slam/image_1', 'camera0/infra2/image_rect_raw'),
            ('visual_slam/camera_info_1', 'camera0/infra2/camera_info'),
            ('visual_slam/imu', 'camera0/imu'),
        ],
    )

    visual_slam_launch_container = ComposableNodeContainer(
        name='visual_slam_launch_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[visual_slam_node],
        output='screen',
    )

    return launch.LaunchDescription([
        camera_serial_numbers_arg,
        num_cameras_arg,
        visual_slam_launch_container,
        imu_horizontal_align_node,
        OpaqueFunction(function=launch_realsense_nodes),
    ])
