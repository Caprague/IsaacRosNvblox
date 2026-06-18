// SPDX-License-Identifier: Apache-2.0
// Mid360 Bridge Node - GPU-accelerated only

#ifndef MID360_TO_NVBLOX_BRIDGE__MID360_BRIDGE_NODE_GPU_HPP_
#define MID360_TO_NVBLOX_BRIDGE__MID360_BRIDGE_NODE_GPU_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "mid360_to_nvblox_bridge/cuda/bridge_converter_gpu.hpp"

#include <vector>
#include <string>
#include <memory>
#include <cmath>

namespace mid360_bridge
{

struct VirtualLidarConfig
{
  int width;
  int height;
  float min_range_m;
  float max_range_m;
  float min_elevation_rad;
  float max_elevation_rad;
  float azimuth_res_rad;
  float elevation_res_rad;
};

class Mid360BridgeNode : public rclcpp::Node
{
public:
  explicit Mid360BridgeNode(const rclcpp::NodeOptions & options);
  ~Mid360BridgeNode() override;

private:
  void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  sensor_msgs::msg::PointCloud2 convertToStructured(
    const sensor_msgs::msg::PointCloud2::SharedPtr & input_cloud);

  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pointcloud_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_structured_;

  // Configuration
  VirtualLidarConfig config_;
  bool enable_hole_filling_;
  int max_hole_fill_iterations_;
  std::string aggregation_method_;

  // GPU converter
  std::unique_ptr<cuda::BridgeConverterGPU> gpu_converter_;

  // Performance monitoring
  bool enable_perf_monitoring_;
  struct PerformanceStats {
    double total_time_ms = 0.0;
    int count = 0;
    double avg_ms() const { return count > 0 ? total_time_ms / count : 0.0; }
  };
  PerformanceStats perf_stats_;
  rclcpp::TimerBase::SharedPtr perf_report_timer_;
  void reportPerformance();
};

}  // namespace mid360_bridge

#endif  // MID360_TO_NVBLOX_BRIDGE__MID360_BRIDGE_NODE_GPU_HPP_
