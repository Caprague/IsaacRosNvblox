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
from launch_ros.actions import Node


def generate_launch_description():
    """Launch file which brings up visual slam node configured for RealSense."""
    realsense_camera_node = Node(
        name='camera',
        namespace='camera',
        package='realsense2_camera',
        executable='realsense2_camera_node',
        parameters=[{
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
            'unite_imu_method': 2
        }],
    )

    # IMU水平校正节点(初始化并发布一个额外frame的TF数据)
    # base_imu_topic: 底盘IMU话题，设置后启用双IMU自动标定camera→base旋转
    #   需要DDS→ROS2桥接节点将Unitree rt/lowstate转为sensor_msgs/Imu
    # camera_base_x/y/z: 相机光学中心在底盘坐标系中的位置
    imu_horizontal_align_node = Node(
        name='imu_horizontal_align_node',
        package='odom_horizontal_transform',
        executable='imu_horizontal_align',
        output='screen',
        parameters=[{
            'start_delay': 2.5,          # 延迟启动水平校正过程
            'base_imu_topic': '',        # 底盘IMU话题，空=禁用双IMU标定
            'camera_base_x': 0.34,       # 相机在底盘坐标系中的X位置
            'camera_base_y': 0.0,        # 相机在底盘坐标系中的Y位置
            'camera_base_z': 0.09,       # 相机在底盘坐标系中的Z位置
        }]
    )

    return launch.LaunchDescription([realsense_camera_node, imu_horizontal_align_node])
