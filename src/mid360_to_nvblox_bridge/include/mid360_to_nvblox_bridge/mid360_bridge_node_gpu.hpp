// SPDX-License-Identifier: Apache-2.0
// Mid360 Bridge Node with GPU Acceleration Support

#ifndef MID360_TO_NVBLOX_BRIDGE__MID360_BRIDGE_NODE_GPU_HPP_
#define MID360_TO_NVBLOX_BRIDGE__MID360_BRIDGE_NODE_GPU_HPP_

#include "mid360_to_nvblox_bridge/mid360_bridge_node.hpp"

#ifdef USE_CUDA
#include "mid360_to_nvblox_bridge/cuda/bridge_converter_gpu.hpp"
#endif

namespace mid360_bridge
{

class Mid360BridgeNodeGPU : public Mid360BridgeNode
{
public:
  explicit Mid360BridgeNodeGPU(const rclcpp::NodeOptions & options);
  ~Mid360BridgeNodeGPU() override;

private:
  // Override conversion function to use GPU
  sensor_msgs::msg::PointCloud2 convertToStructured(
    const sensor_msgs::msg::PointCloud2::SharedPtr & input_cloud) override;
  
  // GPU-accelerated conversion path
  sensor_msgs::msg::PointCloud2 convertToStructuredGPU(
    const sensor_msgs::msg::PointCloud2::SharedPtr & input_cloud);
  
  // CPU fallback
  sensor_msgs::msg::PointCloud2 convertToStructuredCPU(
    const sensor_msgs::msg::PointCloud2::SharedPtr & input_cloud);

#ifdef USE_CUDA
  // GPU converter instance
  std::unique_ptr<cuda::BridgeConverterGPU> gpu_converter_;
#endif
  
  // Configuration
  bool use_gpu_;
  bool gpu_available_;
  size_t gpu_min_points_;  // Use GPU only if point count > threshold
  
  // Performance monitoring
  struct PerformanceStats {
    double total_time_cpu_ms = 0.0;
    double total_time_gpu_ms = 0.0;
    int cpu_count = 0;
    int gpu_count = 0;
    
    double avg_cpu_ms() const { 
      return cpu_count > 0 ? total_time_cpu_ms / cpu_count : 0.0; 
    }
    double avg_gpu_ms() const { 
      return gpu_count > 0 ? total_time_gpu_ms / gpu_count : 0.0; 
    }
    double speedup() const {
      return avg_cpu_ms() > 0 ? avg_cpu_ms() / avg_gpu_ms() : 0.0;
    }
  };
  PerformanceStats perf_stats_;
  
  // Timer for reporting
  rclcpp::TimerBase::SharedPtr perf_report_timer_;
  void reportPerformance();
};

}  // namespace mid360_bridge

#endif  // MID360_TO_NVBLOX_BRIDGE__MID360_BRIDGE_NODE_GPU_HPP_
