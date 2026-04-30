// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2022-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// SPDX-License-Identifier: Apache-2.0

#ifndef NVBLOX_ROS__CONVERSIONS__VERTICAL_RAY_CASTER_HPP_
#define NVBLOX_ROS__CONVERSIONS__VERTICAL_RAY_CASTER_HPP_

#include <nvblox/nvblox.h>
#include "nvblox/core/internal/error_check.h"
#include <memory>
#include <vector>
#include <Eigen/Core>
#include <cuda_runtime.h>
#include <thrust/device_ptr.h>
#include <thrust/host_vector.h>
#include <thrust/device_vector.h>
#include <thrust/sort.h>
#include <thrust/binary_search.h>
#include <rclcpp/rclcpp.hpp>
#include <queue> 
#include <utility> 
#include <vector>

namespace nvblox {
namespace conversions {

/**
 * @class CudaVerticalRayCaster
 * @brief 利用CUDA加速的垂直光线投射采样类，用于从TsdfLayer中采样最近的地形点
 */
class CudaVerticalRayCaster {
public:
  /**
   * @brief 构造函数
   * @param threshold TSDF体素的置信度阈值
   */
  explicit CudaVerticalRayCaster(float threshold = 2.5f);
  
  /**
   * @brief 析构函数
   */
  ~CudaVerticalRayCaster();

  /**
   * @brief 从TsdfLayer中采样最近的地形点
   * @param sample_points_x 采样点的x坐标容器
   * @param sample_points_y 采样点的y坐标容器
   * @param sample_points_z 采样点的z坐标容器
   * @param tsdf_layer TSDF图层
   * @param robot_pose 机器人当前位置和旋转角度
   * @param range_x 采样范围在x轴上的长度
   * @param range_y 采样范围在y轴上的长度
   * @param grid_resolution 采样网格的分辨率
   * @param x_steps 在x轴上的采样步数
   * @param y_steps 在y轴上的采样步数
   * @param x_offset x轴上的偏移量
   * @param y_offset y轴上的偏移量
   * @param z_offset z轴上的偏移量
   * @param max_casting_depth 最大光追采样距离
   * @param cuda_stream CUDA流
   * @return 是否成功执行垂直光线投射采样
   */
  bool sampleTerrainPoints(
    std::vector<float>& sample_points_x,
    std::vector<float>& sample_points_y,
    std::vector<float>& sample_points_z,
    const TsdfLayer& tsdf_layer,
    const Transform& robot_pose,
    const float& range_x,
    const float& range_y,
    const float& grid_resolution,
    const int& x_steps,
    const int& y_steps,
    const float& x_offset,
    const float& y_offset,
    const float& z_offset,
    const float& max_casting_depth,
    const CudaStream& cuda_stream
  );

  /**
   * @brief 设置占据体素占据的权重阈值
   * @param threshold 新的阈值
   */
  void setConfidenceWeightThreshold(float threshold);

  /**
   * @brief 填充无效高程点
   * @param elevation_map 高程地图容器
   * @param valid_mask 有效性掩码容器
   * @param x_steps 在x轴上的采样步数
   * @param y_steps 在y轴上的采样步数
   */
  void fillInvalidElevationPoints(std::vector<float>& elevation_map, 
                            const host_vector<bool>& valid_mask,
                            int x_steps, int y_steps);

private:
  // CPU内存管理
  host_vector<nvblox::Vector3f> h_sample_points_;             // 采样点起始坐标
  host_vector<nvblox::Vector3f> h_terrain_points_;            // 地形点采样点结果
  host_vector<bool> h_point_validity_;                        // 地形点采样点有效性

  // 配置参数
  float confidence_weight_threshold_ = 0.0f;                  // 体素栅格占据的置信权重阈值
  float block_size_ = 0.0f;                                   // 块大小
  float voxel_size_ = 0.0f;                                   // 体素栅格大小
  int total_points_ = 0;                                      // 采样点数量
  float max_distance_ = 0.0f;                                 // 最大光追采样距离
};

}  // namespace conversions
}  // namespace nvblox
        
#endif  // NVBLOX_ROS__CONVERSIONS__VERTICAL_RAY_CASTER_HPP_