// SPDX-License-Identifier: Apache-2.0
// Mid360 to nvblox Bridge Node Implementation

#include "mid360_to_nvblox_bridge/mid360_bridge_node.hpp"
#include <algorithm>
#include <numeric>

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
  this->declare_parameter("output_intensity", false);
  this->declare_parameter("outlier_removal_radius_ratio", 0.1);
  
  // Initialize virtual LiDAR configuration
  config_.width = this->get_parameter("virtual_lidar_width").as_int();
  config_.height = this->get_parameter("virtual_lidar_height").as_int();
  config_.min_range_m = this->get_parameter("min_range_m").as_double();
  config_.max_range_m = this->get_parameter("max_range_m").as_double();
  
  // Convert degrees to radians
  float min_elev_deg = this->get_parameter("min_elevation_deg").as_double();
  float max_elev_deg = this->get_parameter("max_elevation_deg").as_double();
  config_.min_elevation_rad = min_elev_deg * M_PI / 180.0f;
  config_.max_elevation_rad = max_elev_deg * M_PI / 180.0f;
  
  // Compute angular resolutions
  config_.azimuth_res_rad = 2.0f * M_PI / static_cast<float>(config_.width);
  float vertical_fov_rad = config_.max_elevation_rad - config_.min_elevation_rad;
  config_.elevation_res_rad = vertical_fov_rad / static_cast<float>(config_.height - 1);
  
  // Get processing parameters
  enable_hole_filling_ = this->get_parameter("enable_hole_filling").as_bool();
  max_hole_fill_iterations_ = this->get_parameter("max_hole_fill_iterations").as_int();
  aggregation_method_ = this->get_parameter("aggregation_method").as_string();
  output_intensity_ = this->get_parameter("output_intensity").as_bool();
  outlier_removal_radius_ratio_ = this->get_parameter("outlier_removal_radius_ratio").as_double();
  
  // Create subscriber and publisher
  sub_pointcloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "~/input/pointcloud", 10,
    std::bind(&Mid360BridgeNode::pointcloudCallback, this, std::placeholders::_1));
  
  pub_structured_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "~/output/structured_pointcloud", 10);
  
  RCLCPP_INFO(
    this->get_logger(),
    "Mid360 Bridge Node initialized:\n"
    "  Virtual LiDAR: %dx%d (HxV)\n"
    "  Range: [%.2f, %.2f] m\n"
    "  Elevation FOV: [%.1f°, %.1f°]\n"
    "  Angular res: %.2f° (H) x %.2f° (V)\n"
    "  Hole filling: %s",
    config_.width, config_.height,
    config_.min_range_m, config_.max_range_m,
    min_elev_deg, max_elev_deg,
    config_.azimuth_res_rad * 180.0f / M_PI,
    config_.elevation_res_rad * 180.0f / M_PI,
    enable_hole_filling_ ? "enabled" : "disabled");
}

void Mid360BridgeNode::pointcloudCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // Convert to structured format
  auto structured_cloud = convertToStructured(msg);
  
  // Publish
  pub_structured_->publish(structured_cloud);
}

sensor_msgs::msg::PointCloud2 Mid360BridgeNode::convertToStructured(
  const sensor_msgs::msg::PointCloud2::SharedPtr & input_cloud)
{
  // Initialize grid
  std::vector<std::vector<GridCell>> grid(
    config_.height, std::vector<GridCell>(config_.width));
  
  // Iterate through input points and map to grid
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*input_cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*input_cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*input_cloud, "z");
  
  int total_points = 0;
  int mapped_points = 0;
  int out_of_range_points = 0;
  int out_of_fov_points = 0;
  
  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    total_points++;
    
    float x = *iter_x;
    float y = *iter_y;
    float z = *iter_z;
    
    // Skip invalid points
    if (std::isnan(x) || std::isnan(y) || std::isnan(z)) {
      continue;
    }
    
    // Convert to spherical coordinates
    float range, azimuth, elevation;
    if (!cartesianToSpherical(x, y, z, range, azimuth, elevation)) {
      out_of_range_points++;
      continue;
    }
    
    // Map to grid indices
    int u_idx, v_idx;
    if (!sphericalToGrid(azimuth, elevation, u_idx, v_idx)) {
      out_of_fov_points++;
      continue;
    }
    
    // Accumulate depth values (for averaging/aggregation)
    GridCell & cell = grid[v_idx][u_idx];
    cell.depth_sum += range;
    cell.point_count++;
    mapped_points++;
  }
  
  // Aggregate multiple points per cell
  for (int v = 0; v < config_.height; ++v) {
    for (int u = 0; u < config_.width; ++u) {
      GridCell & cell = grid[v][u];
      if (cell.point_count > 0) {
        if (aggregation_method_ == "mean") {
          cell.depth = cell.depth_sum / static_cast<float>(cell.point_count);
        } else if (aggregation_method_ == "min") {
          // Would need to track min during accumulation
          cell.depth = cell.depth_sum / static_cast<float>(cell.point_count);
        } else {
          cell.depth = cell.depth_sum / static_cast<float>(cell.point_count);
        }
        cell.valid = true;
      }
    }
  }
  
  // Optional: Fill holes
  if (enable_hole_filling_) {
    fillHoles(grid);
  }
  
  // Convert grid to PointCloud2 message
  sensor_msgs::msg::PointCloud2 output_cloud;
  output_cloud.header = input_cloud->header;
  output_cloud.height = config_.height;
  output_cloud.width = config_.width;
  output_cloud.is_dense = false;
  output_cloud.is_bigendian = false;
  
  // Define fields
  sensor_msgs::PointCloud2Modifier modifier(output_cloud);
  if (output_intensity_) {
    modifier.setPointCloud2FieldsByString(2, "xyz", "intensity");
  } else {
    modifier.setPointCloud2FieldsByString(1, "xyz");
  }
  
  // Fill in point data
  sensor_msgs::PointCloud2Iterator<float> iter_out_x(output_cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_out_y(output_cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_out_z(output_cloud, "z");
  
  for (int v = 0; v < config_.height; ++v) {
    for (int u = 0; u < config_.width; ++u) {
      const GridCell & cell = grid[v][u];
      
      if (cell.valid) {
        // Convert grid indices back to 3D point
        float azimuth = -M_PI + (u + 0.5f) * config_.azimuth_res_rad;
        float elevation = config_.min_elevation_rad + v * config_.elevation_res_rad;
        float range = cell.depth;
        
        // Spherical to Cartesian
        float cos_elev = cos(elevation);
        *iter_out_x = range * cos_elev * cos(azimuth);
        *iter_out_y = range * cos_elev * sin(azimuth);
        *iter_out_z = range * sin(elevation);
      } else {
        // Invalid point (NaN or very far)
        *iter_out_x = std::numeric_limits<float>::quiet_NaN();
        *iter_out_y = std::numeric_limits<float>::quiet_NaN();
        *iter_out_z = std::numeric_limits<float>::quiet_NaN();
      }
      
      ++iter_out_x;
      ++iter_out_y;
      ++iter_out_z;
    }
  }
  
  RCLCPP_DEBUG(
    this->get_logger(),
    "Conversion stats: %d total, %d mapped (%.1f%%), %d out-of-range, %d out-of-FOV",
    total_points, mapped_points,
    100.0f * mapped_points / std::max(total_points, 1),
    out_of_range_points, out_of_fov_points);
  
  return output_cloud;
}

bool Mid360BridgeNode::cartesianToSpherical(
  float x, float y, float z,
  float & range, float & azimuth, float & elevation) const
{
  // Calculate range
  range = std::sqrt(x * x + y * y + z * z);
  
  // Check range limits
  if (range < config_.min_range_m || range > config_.max_range_m) {
    return false;
  }
  
  // Calculate azimuth (horizontal angle)
  azimuth = std::atan2(y, x);  // [-π, π]
  
  // Calculate elevation (vertical angle from horizontal plane)
  elevation = std::asin(z / range);  // [-π/2, π/2]
  
  return true;
}

bool Mid360BridgeNode::sphericalToGrid(
  float azimuth, float elevation,
  int & u_idx, int & v_idx) const
{
  // Check elevation bounds
  if (elevation < config_.min_elevation_rad || elevation > config_.max_elevation_rad) {
    return false;
  }
  
  // Map azimuth to [0, width)
  // azimuth in [-π, π], map to [0, 2π]
  float azimuth_normalized = azimuth + M_PI;  // [0, 2π]
  u_idx = static_cast<int>(azimuth_normalized / config_.azimuth_res_rad);
  
  // Handle wrap-around
  if (u_idx >= config_.width) {
    u_idx = config_.width - 1;
  }
  if (u_idx < 0) {
    u_idx = 0;
  }
  
  // Map elevation to [0, height)
  float elevation_offset = elevation - config_.min_elevation_rad;
  v_idx = static_cast<int>(elevation_offset / config_.elevation_res_rad);
  
  // Clamp to valid range
  if (v_idx < 0) v_idx = 0;
  if (v_idx >= config_.height) v_idx = config_.height - 1;
  
  return true;
}

void Mid360BridgeNode::fillHoles(std::vector<std::vector<GridCell>> & grid)
{
  // Simple hole filling: interpolate from neighbors
  for (int iter = 0; iter < max_hole_fill_iterations_; ++iter) {
    int filled_count = 0;
    
    for (int v = 0; v < config_.height; ++v) {
      for (int u = 0; u < config_.width; ++u) {
        if (!grid[v][u].valid) {
          interpolateCell(grid, u, v);
          if (grid[v][u].valid) {
            filled_count++;
          }
        }
      }
    }
    
    if (filled_count == 0) {
      break;  // No more holes filled
    }
    
    RCLCPP_DEBUG(
      this->get_logger(),
      "Hole filling iteration %d: filled %d cells", iter + 1, filled_count);
  }
}

void Mid360BridgeNode::interpolateCell(
  std::vector<std::vector<GridCell>> & grid,
  int u, int v)
{
  // Check 4-connected neighbors
  std::vector<float> neighbor_depths;
  
  // Left
  if (u > 0 && grid[v][u - 1].valid) {
    neighbor_depths.push_back(grid[v][u - 1].depth);
  }
  // Right
  if (u < config_.width - 1 && grid[v][u + 1].valid) {
    neighbor_depths.push_back(grid[v][u + 1].depth);
  }
  // Top
  if (v > 0 && grid[v - 1][u].valid) {
    neighbor_depths.push_back(grid[v - 1][u].depth);
  }
  // Bottom
  if (v < config_.height - 1 && grid[v + 1][u].valid) {
    neighbor_depths.push_back(grid[v + 1][u].depth);
  }
  
  // Also check horizontal wrap-around for azimuth
  int u_wrap_left = (u == 0) ? config_.width - 1 : u - 1;
  int u_wrap_right = (u == config_.width - 1) ? 0 : u + 1;
  if (u == 0 && grid[v][u_wrap_left].valid) {
    neighbor_depths.push_back(grid[v][u_wrap_left].depth);
  }
  if (u == config_.width - 1 && grid[v][u_wrap_right].valid) {
    neighbor_depths.push_back(grid[v][u_wrap_right].depth);
  }
  
  // Need at least 2 neighbors for interpolation
  if (neighbor_depths.size() >= 2) {
    // Average of neighbors
    float sum = std::accumulate(neighbor_depths.begin(), neighbor_depths.end(), 0.0f);
    grid[v][u].depth = sum / static_cast<float>(neighbor_depths.size());
    grid[v][u].valid = true;
  }
}

}  // namespace mid360_bridge

// Register as composable node
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(mid360_bridge::Mid360BridgeNode)
