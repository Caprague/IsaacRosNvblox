// SPDX-License-Identifier: Apache-2.0
// Mid360 to nvblox Bridge Node
// Converts Mid360 unstructured pointcloud to structured grid for nvblox

#ifndef MID360_TO_NVBLOX_BRIDGE__MID360_BRIDGE_NODE_HPP_
#define MID360_TO_NVBLOX_BRIDGE__MID360_BRIDGE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <vector>
#include <cmath>
#include <memory>

namespace mid360_bridge
{

/// Configuration for the virtual LiDAR model
struct VirtualLidarConfig
{
  int width;                      // Azimuth divisions (horizontal resolution)
  int height;                     // Elevation divisions (vertical beams)
  float min_range_m;              // Minimum valid range
  float max_range_m;              // Maximum valid range
  float min_elevation_rad;        // Below zero (negative value)
  float max_elevation_rad;        // Above zero (positive value)
  float azimuth_res_rad;          // Computed: 2π / width
  float elevation_res_rad;        // Computed: FOV / (height - 1)
};

/// Represents a grid cell in the structured pointcloud
struct GridCell
{
  float depth;                    // Range value
  bool valid;                     // Whether this cell has data
  int point_count;                // Number of points mapped to this cell
  float depth_sum;                // Sum for averaging
  
  GridCell() : depth(0.0f), valid(false), point_count(0), depth_sum(0.0f) {}
};

class Mid360BridgeNode : public rclcpp::Node
{
public:
  explicit Mid360BridgeNode(const rclcpp::NodeOptions & options);
  ~Mid360BridgeNode() = default;

private:
  // Callback for Mid360 pointcloud
  void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

protected:
  // Convert unstructured to structured pointcloud
  virtual sensor_msgs::msg::PointCloud2 convertToStructured(
    const sensor_msgs::msg::PointCloud2::SharedPtr & input_cloud);
  
  // Convert 3D point to spherical coordinates
  bool cartesianToSpherical(
    float x, float y, float z,
    float & range, float & azimuth, float & elevation) const;
  
  // Map spherical coordinates to grid indices
  bool sphericalToGrid(
    float azimuth, float elevation,
    int & u_idx, int & v_idx) const;
  
  // Fill holes in the grid (optional)
  void fillHoles(std::vector<std::vector<GridCell>> & grid);
  
  // Interpolate missing cells
  void interpolateCell(
    std::vector<std::vector<GridCell>> & grid,
    int u, int v);

  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pointcloud_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_structured_;
  
  // Virtual LiDAR configuration
  VirtualLidarConfig config_;
  
  // Processing parameters
  bool enable_hole_filling_;
  int max_hole_fill_iterations_;
  std::string aggregation_method_;  // "mean", "min", "max", "median"
  bool output_intensity_;
  float outlier_removal_radius_ratio_;
};

}  // namespace mid360_bridge

#endif  // MID360_TO_NVBLOX_BRIDGE__MID360_BRIDGE_NODE_HPP_
