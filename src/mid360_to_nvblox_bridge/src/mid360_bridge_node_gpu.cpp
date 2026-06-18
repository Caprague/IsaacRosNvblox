// SPDX-License-Identifier: Apache-2.0
// Mid360 Bridge Node - GPU-accelerated implementation

#include "mid360_to_nvblox_bridge/mid360_bridge_node_gpu.hpp"
#include <chrono>

namespace mid360_bridge
{

Mid360BridgeNode::Mid360BridgeNode(const rclcpp::NodeOptions & options)
: Node("mid360_bridge_node", options)
{
  // Declare and get parameters
  this->declare_parameter("virtual_lidar_width", 1800);
  this->declare_parameter("virtual_lidar_height", 32);
  this->declare_parameter("min_range_m", 0.5);
  this->declare_parameter("max_range_m", 30.0);
  this->declare_parameter("min_elevation_deg", -7.0);
  this->declare_parameter("max_elevation_deg", 52.0);
  this->declare_parameter("enable_hole_filling", true);
  this->declare_parameter("max_hole_fill_iterations", 2);
  this->declare_parameter("aggregation_method", "mean");
  this->declare_parameter("enable_perf_monitoring", true);

  // Initialize virtual LiDAR configuration
  config_.width = this->get_parameter("virtual_lidar_width").as_int();
  config_.height = this->get_parameter("virtual_lidar_height").as_int();
  config_.min_range_m = this->get_parameter("min_range_m").as_double();
  config_.max_range_m = this->get_parameter("max_range_m").as_double();

  float min_elev_deg = this->get_parameter("min_elevation_deg").as_double();
  float max_elev_deg = this->get_parameter("max_elevation_deg").as_double();
  config_.min_elevation_rad = min_elev_deg * M_PI / 180.0f;
  config_.max_elevation_rad = max_elev_deg * M_PI / 180.0f;
  config_.azimuth_res_rad = 2.0f * M_PI / static_cast<float>(config_.width);
  config_.elevation_res_rad =
    (config_.max_elevation_rad - config_.min_elevation_rad) / static_cast<float>(config_.height - 1);

  enable_hole_filling_ = this->get_parameter("enable_hole_filling").as_bool();
  max_hole_fill_iterations_ = this->get_parameter("max_hole_fill_iterations").as_int();
  aggregation_method_ = this->get_parameter("aggregation_method").as_string();
  enable_perf_monitoring_ = this->get_parameter("enable_perf_monitoring").as_bool();

  // Create subscriber and publisher
  sub_pointcloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "~/input/pointcloud", 10,
    std::bind(&Mid360BridgeNode::pointcloudCallback, this, std::placeholders::_1));

  pub_structured_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "~/output/structured_pointcloud", 10);

  // Initialize GPU converter
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
    "Mid360 Bridge Node initialized (GPU):\n"
    "  Virtual LiDAR: %dx%d (HxV)\n"
    "  Range: [%.2f, %.2f] m\n"
    "  Elevation FOV: [%.1f°, %.1f°]\n"
    "  Angular res: %.2f° (H) x %.2f° (V)\n"
    "  Hole filling: %s\n"
    "  GPU: %s",
    config_.width, config_.height,
    config_.min_range_m, config_.max_range_m,
    min_elev_deg, max_elev_deg,
    config_.azimuth_res_rad * 180.0f / M_PI,
    config_.elevation_res_rad * 180.0f / M_PI,
    enable_hole_filling_ ? "enabled" : "disabled",
    cuda::getCudaDeviceInfo().c_str());

  // Setup performance monitoring
  if (enable_perf_monitoring_) {
    perf_report_timer_ = this->create_wall_timer(
      std::chrono::seconds(10),
      std::bind(&Mid360BridgeNode::reportPerformance, this));
  }
}

Mid360BridgeNode::~Mid360BridgeNode()
{
  gpu_converter_.reset();
}

void Mid360BridgeNode::pointcloudCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  auto structured_cloud = convertToStructured(msg);
  pub_structured_->publish(structured_cloud);
}

sensor_msgs::msg::PointCloud2 Mid360BridgeNode::convertToStructured(
  const sensor_msgs::msg::PointCloud2::SharedPtr & input_cloud)
{
  auto start = std::chrono::high_resolution_clock::now();
  const size_t num_points = input_cloud->width * input_cloud->height;

  // Extract XYZ data
  std::vector<float> input_x, input_y, input_z;
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

  gpu_converter_->convertPointcloud(
    input_x, input_y, input_z,
    output_x, output_y, output_z,
    method,
    max_hole_fill_iterations_);

  // Package into PointCloud2 message
  sensor_msgs::msg::PointCloud2 output_cloud;
  output_cloud.header = input_cloud->header;
  output_cloud.header.frame_id = "lidar";
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
  perf_stats_.total_time_ms += elapsed_ms;
  perf_stats_.count++;

  RCLCPP_DEBUG(
    this->get_logger(),
    "GPU conversion: %zu points -> %zu cells in %.2f ms",
    num_points, output_x.size(), elapsed_ms);

  return output_cloud;
}

void Mid360BridgeNode::reportPerformance()
{
  if (perf_stats_.count == 0) {
    return;
  }

  RCLCPP_INFO(
    this->get_logger(),
    "\n=== Performance Report ===\n"
    "  %d frames, avg %.2f ms/frame\n"
    "===========================",
    perf_stats_.count, perf_stats_.avg_ms());

  perf_stats_ = PerformanceStats();
}

}  // namespace mid360_bridge

// Register as composable node
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(mid360_bridge::Mid360BridgeNode)
