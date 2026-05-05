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

#ifndef NVBLOX_ROS__LAYER_PUBLISHING_HPP_
#define NVBLOX_ROS__LAYER_PUBLISHING_HPP_

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <nvblox_msgs/msg/voxel_block_layer.hpp>
#include "nvblox/nvblox.h"
#include "nvblox_msgs/msg/mesh.hpp"
#include "nvblox/serialization/layer_serializer_gpu.h"
#include "nvblox_ros/conversions/pointcloud_conversions.hpp"
#include "nvblox_ros/conversions/vertical_ray_caster.hpp"

#include <std_msgs/msg/float32_multi_array.hpp>

namespace nvblox
{

/// Publishing of nvblox block layers
class LayerPublisher
{
public:
  /// Create a layer publisher.
  ///
  /// @param mapping_type Mapping type, used to determine which topics to advertise
  /// @param min_tsdf_weight Min weight for visualized TSDF voxels
  /// @param node ROS node
  LayerPublisher(
    const MappingType mapping_type, const float min_tsdf_weight,
    const float exclusion_height_m, const float exclusion_radius_m, rclcpp::Node * node);

  /// Serialize and publish all layers that have active subscribers
  ///
  /// @param T_L_C Transformation used for radial exclusion
  /// @param frame_id Frame id for the published visualization topics
  /// @param timestamp Timestamp for the published visualization topics
  /// @param static_mapper Static mapper
  /// @param dynamic_mapper Dynamic mapper. Can be nullptr if not available
  /// @param logger ROS logger
  void serializeAndpublishSubscribedLayers(
    const Transform & T_L_C, const std::string & frame_id,
    rclcpp::Time timestamp, const float layer_streamer_bandwidth_limit_mbps,
    std::shared_ptr<Mapper> static_mapper,
    std::shared_ptr<Mapper> dynamic_mapper, const rclcpp::Logger & logger);

  /// Serialize and publish mesh layer and height scan data, when any have active subscribers
  /// @param height_data_out Output: robot-relative height values for stats logging by caller
  void publishLocomotionHeightScan(
    const Transform & T_L_C, const std::string & frame_id,
    rclcpp::Time timestamp, const float layer_streamer_bandwidth_limit_mbps,
    std::shared_ptr<Mapper> static_mapper,
    std::shared_ptr<Mapper> dynamic_mapper,
    const rclcpp::Logger & logger,
    const CudaStream& cuda_stream,
    std::vector<float>* height_data_out = nullptr);

  /// Publish a cached/compensated locomotion height scan when TSDF lock is contended.
  /// @param height_data Compensated robot-relative height values
  /// @param frame_id Frame id for the published message
  /// @param timestamp Timestamp for the published message
  void publishCachedLocomotionHeightScan(
    const std::vector<float>& height_data,
    const std::string& frame_id,
    const rclcpp::Time& timestamp);

  /// Serialize and publish mesh layer and height scan data, when any have active subscribers
  ///
  /// @param T_L_C Transformation used for radial exclusion
  /// @param frame_id Frame id for the published visualization topics
  /// @param timestamp Timestamp for the published visualization topics
  /// @param static_mapper Static mapper
  /// @param dynamic_mapper Dynamic mapper. Can be nullptr if not available
  /// @param logger ROS logger
  void publishNavigationHeightScan(
    const Transform & T_L_C, const std::string & frame_id,
    rclcpp::Time timestamp, const float layer_streamer_bandwidth_limit_mbps,
    std::shared_ptr<Mapper> static_mapper,
    std::shared_ptr<Mapper> dynamic_mapper,
    const rclcpp::Logger & logger,
    const CudaStream& cuda_stream);

  /// 检查是否需要进行地平面初始化（委托给ray_caster_）
  bool needsGroundPlaneInit() const {
    return ray_caster_ && ray_caster_->needsGroundPlaneInit();
  }

  /// 执行地平面初始化（需在持有unique_lock时调用，委托给ray_caster_）
  void initializeGroundPlaneIfNeeded(
      TsdfLayer& tsdf_layer,
      const Transform& robot_pose,
      const CudaStream& cuda_stream) {
    if (ray_caster_ && ray_caster_->needsGroundPlaneInit()) {
      ray_caster_->initializeGroundPlane(tsdf_layer, robot_pose, cuda_stream);
    }
  }

private:
  /// Determine which layer should be streamed based on active subscribers
  LayerTypeBitMask getLayersToStreamBitMask();

  /// Update and publish the mesh
  void publishMesh(
    std::shared_ptr<const SerializedMeshLayer> serialized_mesh,
    const std::vector<Index3D> & blocks_to_remove,
    const float block_size, const std::string & frame_id,
    const rclcpp::Time & timestamp, const rclcpp::Logger & logger);

  /// Implementation of publishLocomotionHeightScan
  /// @param height_data_out Output: robot-relative height values
  void publishLocomotionHeightScan_impl(
    TsdfLayer& tsdf_layer,
    const Transform& base_pose,
    const rclcpp::Time& timestamp,
    const rclcpp::Logger& logger,
    const CudaStream& cuda_stream,
    std::vector<float>* height_data_out = nullptr);
  
  /// Implementation of publishNavigationHeightScan
  void publishNavigationHeightScan_impl(
    TsdfLayer& tsdf_layer,  // 改为非const，因为可能需要初始化地平面
    const Transform& base_pose,
    const rclcpp::Logger& logger,
    const CudaStream& cuda_stream);

  // Cache the last known number of subscribers.
  size_t mesh_subscriber_count_ = 0;

  // Params
  float min_tsdf_weight_ = 0;
  float exclusion_height_m_ = -1.0;
  float exclusion_radius_m_ = -1.0;

  // Publishers using nvblox plugin. Allows for bandwidth limitation.
  rclcpp::Publisher<nvblox_msgs::msg::Mesh>::SharedPtr mesh_publisher_;
  rclcpp::Publisher<nvblox_msgs::msg::VoxelBlockLayer>::SharedPtr
    tsdf_layer_publisher_plugin_;
  rclcpp::Publisher<nvblox_msgs::msg::VoxelBlockLayer>::SharedPtr
    color_layer_publisher_plugin_;
  rclcpp::Publisher<nvblox_msgs::msg::VoxelBlockLayer>::SharedPtr
    freespace_layer_publisher_plugin_;
  rclcpp::Publisher<nvblox_msgs::msg::VoxelBlockLayer>::SharedPtr
    dynamic_occupancy_layer_publisher_plugin_;


  // Publishers using markers (for fallback). The whole layer will be transmitted every time.
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
    tsdf_layer_publisher_marker_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
    color_layer_publisher_marker_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
    freespace_layer_publisher_marker_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
    dynamic_occupancy_layer_publisher_marker_;

  // 全局采样点坐标及高程
  std::vector<float> locomotion_sample_points_x;
  std::vector<float> locomotion_sample_points_y;
  std::vector<float> locomotion_sample_points_z;
  std::vector<float> navigation_sample_points_x;
  std::vector<float> navigation_sample_points_y;
  std::vector<float> navigation_sample_points_z;

  // CUDA 垂直光线投射采样器（复用实例，避免频繁创建销毁）
  std::unique_ptr<conversions::CudaVerticalRayCaster> ray_caster_;

  // heightscan 发布
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr locomotion_hs_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr navigation_hs_publisher_;

  // 可视化发布
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr locomotion_hs_pc_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr navigation_hs_pc_publisher_;
};

}  // namespace nvblox

#endif  // NVBLOX_ROS__LAYER_PUBLISHING_HPP_
