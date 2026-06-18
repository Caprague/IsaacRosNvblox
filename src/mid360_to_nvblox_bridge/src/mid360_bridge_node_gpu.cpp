// SPDX-License-Identifier: Apache-2.0
// Implementation of GPU-accelerated Bridge Node

#include "mid360_to_nvblox_bridge/mid360_bridge_node_gpu.hpp"
#include <chrono>

namespace mid360_bridge
{

Mid360BridgeNodeGPU::Mid360BridgeNodeGPU(const rclcpp::NodeOptions & options)
: Mid360BridgeNode(options)
{
  // Declare GPU-specific parameters
  this->declare_parameter("use_gpu", true);
  this->declare_parameter("gpu_min_points", 10000);
  this->declare_parameter("enable_perf_monitoring", true);
  
  use_gpu_ = this->get_parameter("use_gpu").as_bool();
  gpu_min_points_ = this->get_parameter("gpu_min_points").as_int();
  
#ifdef USE_CUDA
  // Check CUDA availability
  gpu_available_ = cuda::isCudaAvailable();
  
  if (gpu_available_ && use_gpu_) {
    try {
      // Create GPU converter
      cuda::VirtualLidarConfig gpu_config;
      gpu_config.width = config_.width;
      gpu_config.height = config_.height;
      gpu_config.min_range_m = config_.min_range_m;
      gpu_config.max_range_m = config_.max_range_m;
      gpu_config.min_elevation_rad = config_.min_elevation_rad;
      gpu_config.max_elevation_rad = config_.max_elevation_rad;
      gpu_config.azimuth_res_rad = config_.azimuth_res_rad;
      gpu_config.elevation_res_rad = config_.elevation_res_rad;
      
      gpu_converter_ = std::make_unique<cuda::BridgeConverterGPU>(gpu_config);
      
      RCLCPP_INFO(
        this->get_logger(),
        "GPU acceleration ENABLED\n%s\n"
        "GPU will be used for pointclouds with >%zu points",
        cuda::getCudaDeviceInfo().c_str(),
        gpu_min_points_);
      
    } catch (const std::exception& e) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to initialize GPU converter: %s\nFalling back to CPU",
        e.what());
      gpu_available_ = false;
    }
  } else {
    RCLCPP_INFO(this->get_logger(), "GPU acceleration DISABLED (running on CPU)");
  }
#else
  gpu_available_ = false;
  RCLCPP_INFO(this->get_logger(), 
    "Built without CUDA support - using CPU only");
#endif

  // Setup performance monitoring
  if (this->get_parameter("enable_perf_monitoring").as_bool()) {
    perf_report_timer_ = this->create_wall_timer(
      std::chrono::seconds(10),
      std::bind(&Mid360BridgeNodeGPU::reportPerformance, this));
  }
}

Mid360BridgeNodeGPU::~Mid360BridgeNodeGPU()
{
#ifdef USE_CUDA
  gpu_converter_.reset();
#endif
}

sensor_msgs::msg::PointCloud2 Mid360BridgeNodeGPU::convertToStructured(
  const sensor_msgs::msg::PointCloud2::SharedPtr & input_cloud)
{
  const size_t num_points = input_cloud->width * input_cloud->height;
  
  // Decide CPU vs GPU based on point count and availability
  bool use_gpu_path = gpu_available_ && use_gpu_ && 
                      (num_points >= gpu_min_points_);
  
  if (use_gpu_path) {
#ifdef USE_CUDA
    return convertToStructuredGPU(input_cloud);
#else
    return convertToStructuredCPU(input_cloud);
#endif
  } else {
    return convertToStructuredCPU(input_cloud);
  }
}

#ifdef USE_CUDA
sensor_msgs::msg::PointCloud2 Mid360BridgeNodeGPU::convertToStructuredGPU(
  const sensor_msgs::msg::PointCloud2::SharedPtr & input_cloud)
{
  auto start = std::chrono::high_resolution_clock::now();
  
  // Extract XYZ data from PointCloud2
  std::vector<float> input_x, input_y, input_z;
  const size_t num_points = input_cloud->width * input_cloud->height;
  input_x.reserve(num_points);
  input_y.reserve(num_points);
  input_z.reserve(num_points);
  
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*input_cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*input_cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*input_cloud, "z");
  
  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    input_x.push_back(*iter_x);
    input_y.push_back(*iter_y);
    input_z.push_back(*iter_z);
  }
  
  // Run GPU conversion
  std::vector<float> output_x, output_y, output_z;
  
  cuda::AggregationMethod method = cuda::AggregationMethod::MEAN;
  if (aggregation_method_ == "min") {
    method = cuda::AggregationMethod::MIN;
  } else if (aggregation_method_ == "max") {
    method = cuda::AggregationMethod::MAX;
  }
  
  try {
    gpu_converter_->convertPointcloud(
      input_x, input_y, input_z,
      output_x, output_y, output_z,
      method,
      max_hole_fill_iterations_
    );
  } catch (const std::exception& e) {
    RCLCPP_ERROR(
      this->get_logger(),
      "GPU conversion failed: %s\nFalling back to CPU",
      e.what());
    return convertToStructuredCPU(input_cloud);
  }
  
  // Package into PointCloud2 message
  sensor_msgs::msg::PointCloud2 output_cloud;
  output_cloud.header = input_cloud->header;
  output_cloud.height = config_.height;
  output_cloud.width = config_.width;
  output_cloud.is_dense = false;
  
  sensor_msgs::PointCloud2Modifier modifier(output_cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(output_x.size());
  
  sensor_msgs::PointCloud2Iterator<float> iter_out_x(output_cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_out_y(output_cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_out_z(output_cloud, "z");
  
  for (size_t i = 0; i < output_x.size(); ++i) {
    *iter_out_x = output_x[i];
    *iter_out_y = output_y[i];
    *iter_out_z = output_z[i];
    ++iter_out_x;
    ++iter_out_y;
    ++iter_out_z;
  }
  
  // Record performance
  auto end = std::chrono::high_resolution_clock::now();
  double elapsed_ms = std::chrono::duration<double, std::milli>(end - start).count();
  perf_stats_.total_time_gpu_ms += elapsed_ms;
  perf_stats_.gpu_count++;
  
  RCLCPP_DEBUG(
    this->get_logger(),
    "GPU conversion: %zu points → %zu valid cells in %.2f ms",
    num_points, output_x.size(), elapsed_ms);
  
  return output_cloud;
}
#endif

sensor_msgs::msg::PointCloud2 Mid360BridgeNodeGPU::convertToStructuredCPU(
  const sensor_msgs::msg::PointCloud2::SharedPtr & input_cloud)
{
  auto start = std::chrono::high_resolution_clock::now();
  
  // Call parent class CPU implementation
  auto output = Mid360BridgeNode::convertToStructured(input_cloud);
  
  auto end = std::chrono::high_resolution_clock::now();
  double elapsed_ms = std::chrono::duration<double, std::milli>(end - start).count();
  perf_stats_.total_time_cpu_ms += elapsed_ms;
  perf_stats_.cpu_count++;
  
  return output;
}

void Mid360BridgeNodeGPU::reportPerformance()
{
  if (perf_stats_.cpu_count == 0 && perf_stats_.gpu_count == 0) {
    return;
  }
  
  RCLCPP_INFO(
    this->get_logger(),
    "\n"
    "=== Performance Report (Last 10s) ===\n"
    "CPU: %d frames, avg %.2f ms/frame\n"
    "GPU: %d frames, avg %.2f ms/frame\n"
    "Speedup: %.1fx\n"
    "====================================",
    perf_stats_.cpu_count, perf_stats_.avg_cpu_ms(),
    perf_stats_.gpu_count, perf_stats_.avg_gpu_ms(),
    perf_stats_.speedup());
  
  // Reset stats
  perf_stats_ = PerformanceStats();
}

}  // namespace mid360_bridge

// Register as composable node
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(mid360_bridge::Mid360BridgeNodeGPU)
