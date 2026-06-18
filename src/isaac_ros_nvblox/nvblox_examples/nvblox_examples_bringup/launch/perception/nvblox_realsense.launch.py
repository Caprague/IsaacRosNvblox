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

from typing import List, Tuple

from launch import Action, LaunchDescription
from launch_ros.descriptions import ComposableNode
import isaac_ros_launch_utils as lu

from nvblox_ros_python_utils.nvblox_launch_utils import NvbloxMode, NvbloxCamera
from nvblox_ros_python_utils.nvblox_constants import NVBLOX_CONTAINER_NAME


def get_realsense_remappings(mode: NvbloxMode, num_cameras: int = 1) -> List[Tuple[str, str]]:
    # NOTE(xinjieyao, 04.09.2024): Current in this function we only support:
    # - On/off emitter flashing + realsense_splitter on camera_0 (front camera).
    # - (Optional) people segmentation on all cameras.
    # - (Optional) people detection on all cameras.

    remappings = []
    assert (num_cameras == 1) 
    remappings.append((f'camera_0/depth/image', 'camera/depth/image_rect_raw'))
    remappings.append((f'camera_0/depth/camera_info', 'camera/depth/camera_info'))
    remappings.append((f'camera_0/color/image', 'camera/color/image_raw'))
    remappings.append((f'camera_0/color/camera_info', 'camera/color/camera_info'))

    return remappings

def add_nvblox(args: lu.ArgumentContainer) -> List[Action]:

    mode = NvbloxMode[args.mode]
    camera = NvbloxCamera[args.camera]
    num_cameras = int(args.num_cameras)
    use_lidar = lu.is_true(args.lidar)

    if camera == NvbloxCamera.realsense:
        assert args.num_cameras == 1, 'NvbloxCamera.realsense shall only be set for num_cameras==1'

    base_config = lu.get_path('nvblox_examples_bringup', 'config/nvblox/nvblox_realsense_base.yaml')
    segmentation_config = lu.get_path('nvblox_examples_bringup',
                                      'config/nvblox/specializations/nvblox_segmentation.yaml')
    detection_config = lu.get_path('nvblox_examples_bringup',
                                   'config/nvblox/specializations/nvblox_detection.yaml')
    dynamics_config = lu.get_path('nvblox_examples_bringup',
                                  'config/nvblox/specializations/nvblox_dynamics.yaml')
    realsense_config = lu.get_path('nvblox_examples_bringup',
                                   'config/nvblox/specializations/nvblox_realsense.yaml')
    multi_realsense_config = lu.get_path(
        'nvblox_examples_bringup', 'config/nvblox/specializations/nvblox_multi_realsense.yaml')
    
    if mode is NvbloxMode.static:
        mode_config = {}
    elif mode is NvbloxMode.people_segmentation:
        mode_config = segmentation_config
        assert not use_lidar, 'Can not run lidar with people segmentation mode.'
    elif mode is NvbloxMode.people_detection:
        mode_config = detection_config
        assert not use_lidar, 'Can not run lidar with people detection mode.'
    elif mode is NvbloxMode.dynamic:
        mode_config = dynamics_config
        assert not use_lidar, 'Can not run lidar with dynamic mode.'
    else:
        raise Exception(f'Mode {mode} not implemented for nvblox.')

    if camera is NvbloxCamera.realsense:
        remappings = get_realsense_remappings(mode, num_cameras)
        camera_config = realsense_config
        assert not use_lidar, 'Can not run lidar for realsense example.'
    elif camera is NvbloxCamera.multi_realsense:
        remappings = get_realsense_remappings(mode, num_cameras)
        camera_config = multi_realsense_config
        assert not use_lidar, 'Can not run lidar for multi realsense example.'
    else:
        raise Exception(f'Camera {camera} not implemented for nvblox.')

    parameters = []
    parameters.append(base_config)
    parameters.append(mode_config)
    parameters.append(camera_config)
    parameters.append({'num_cameras': num_cameras})
    parameters.append({'use_lidar': use_lidar})

    # Add the nvblox node.
    nvblox_node = ComposableNode(
        name='nvblox_node',
        package='nvblox_ros',
        plugin='nvblox::NvbloxNode',
        remappings=remappings,
        parameters=parameters,
    )

    actions = []
    if args.run_standalone:
        actions.append(lu.component_container(args.container_name))
    actions.append(lu.load_composable_nodes(args.container_name, [nvblox_node]))
    actions.append(
        lu.log_info(
            ["Starting nvblox with the '",
             str(camera), "' camera in '",
             str(mode), "' mode."]))
    return actions


def generate_launch_description() -> LaunchDescription:
    args = lu.ArgumentContainer()
    args.add_arg('mode')
    args.add_arg('camera', 1)
    args.add_arg('num_cameras', 1)
    args.add_arg('lidar', 'True')
    args.add_arg('container_name', NVBLOX_CONTAINER_NAME)
    args.add_arg('run_standalone', 'False')

    args.add_opaque_function(add_nvblox)
    return LaunchDescription(args.get_launch_actions())
