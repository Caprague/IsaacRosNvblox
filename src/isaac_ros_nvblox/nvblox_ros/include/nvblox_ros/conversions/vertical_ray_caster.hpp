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

#ifndef NVBLOX_ROS__CONVERSIONS__VERTICAL_RAY_CASTER_H_
#define NVBLOX_ROS__CONVERSIONS__VERTICAL_RAY_CASTER_H_

#include <nvblox/nvblox.h>
#include <memory>
#include <vector>
#include <Eigen/Core>
#include <nvblox/core/cuda_stream.h>

namespace nvblox {
namespace conversions {

/**
 * @class CudaVerticalRayCaster
 * @brief 利用CUDA加速的垂直光线投射采样类，用于从TsdfLayer中采样最近的地形点
 *
 * 该类使用独立的非阻塞CUDA流执行光线追踪核函数，避免与主映射流(integrateDepth,
 * updateEsdf等)产生阻塞。getGpuLayerView()仍在主流上执行(确保GPU哈希是最新的)，
 * 其后的核函数启动和D2H传输在独立流上异步执行。
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
   * @param cuda_stream 主CUDA流(用于getGpuLayerView更新GPU哈希)
   * @return 是否成功执行垂直光线投射采样
   */
  bool sampleTerrainPoints(
    std::vector<float>& sample_points_x,
    std::vector<float>& sample_points_y,
    std::vector<float>& sample_points_z,
    TsdfLayer& tsdf_layer,  // 改为非const，因为可能需要初始化地平面
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

  /**
   * @brief 初始化地平面体素栅格
   * @param tsdf_layer TSDF图层（需要非const，因为要修改）
   * @param robot_pose 机器人当前位置和旋转角度
   * @param cuda_stream CUDA流
   * @return 是否成功初始化
   */
  bool initializeGroundPlane(TsdfLayer& tsdf_layer,
                            const Transform& robot_pose,
                            const CudaStream& cuda_stream);

  /**
   * @brief 检查是否需要进行地平面初始化
   * @return true 如果尚未初始化且采样次数已达到延迟阈值
   */
  bool needsGroundPlaneInit() const {
    return !ground_plane_initialized_ && sample_count_ >= kGroundPlaneInitDelay;
  }

  /**
   * @brief 标记地平面已初始化（供外部调用者在独立初始化后使用）
   */
  void markGroundPlaneInitialized() {
    ground_plane_initialized_ = true;
  }

private:
  // CPU内存管理
  host_vector<nvblox::Vector3f> h_terrain_points_;            // 地形点采样结果
  host_vector<bool> h_point_validity_;                        // 地形点采样有效性

  // 独立非阻塞CUDA流 —— 用于光线追踪核函数和D2H传输，避免与主映射流阻塞
  CudaStreamOwning height_scan_stream_{cudaStreamNonBlocking};

  // 配置参数
  float confidence_weight_threshold_ = 0.0f;                  // 体素栅格占据的置信权重阈值
  float block_size_ = 0.0f;                                   // 块大小
  float voxel_size_ = 0.0f;                                   // 体素栅格大小
  int total_points_ = 0;                                      // 采样点数量
  float max_distance_ = 0.0f;                                 // 最大光追采样距离

  // 地平面初始化参数
  int sample_count_ = 0;                                      // 采样计数器
  static constexpr int kGroundPlaneInitDelay = 100;            // 地平面初始化延迟采样次数
  bool ground_plane_initialized_ = false;                     // 地平面是否已初始化
};

}  // namespace conversions
}  // namespace nvblox

#endif  // NVBLOX_ROS__CONVERSIONS__VERTICAL_RAY_CASTER_H_