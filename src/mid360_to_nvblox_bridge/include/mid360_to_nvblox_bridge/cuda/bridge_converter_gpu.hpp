// SPDX-License-Identifier: Apache-2.0
// Host-side declarations for GPU Bridge Converter
// This header is safe to include from .cpp files (no CUDA dependencies)

#ifndef MID360_TO_NVBLOX_BRIDGE__CUDA__BRIDGE_CONVERTER_GPU_HPP_
#define MID360_TO_NVBLOX_BRIDGE__CUDA__BRIDGE_CONVERTER_GPU_HPP_

#include <vector>
#include <string>
#include <memory>
#include <cstddef>

namespace mid360_bridge {
namespace cuda {

struct VirtualLidarConfig {
  int width;
  int height;
  float min_range_m;
  float max_range_m;
  float min_elevation_rad;
  float max_elevation_rad;
  float azimuth_res_rad;
  float elevation_res_rad;
};

enum class AggregationMethod {
  MEAN = 0,
  MIN = 1,
  MAX = 2
};

class BridgeConverterGPU {
public:
  explicit BridgeConverterGPU(const VirtualLidarConfig& config);
  ~BridgeConverterGPU();

  BridgeConverterGPU(const BridgeConverterGPU&) = delete;
  BridgeConverterGPU& operator=(const BridgeConverterGPU&) = delete;

  void convertPointcloud(
      const std::vector<float>& input_x,
      const std::vector<float>& input_y,
      const std::vector<float>& input_z,
      std::vector<float>& output_x,
      std::vector<float>& output_y,
      std::vector<float>& output_z,
      AggregationMethod aggregation = AggregationMethod::MEAN,
      int hole_fill_iterations = 2);

  void convertToDepthImage(
      const std::vector<float>& input_x,
      const std::vector<float>& input_y,
      const std::vector<float>& input_z,
      std::vector<float>& output_depth_image,
      AggregationMethod aggregation = AggregationMethod::MEAN,
      int hole_fill_iterations = 2);

  int getValidPointCount();

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

bool isCudaAvailable();
std::string getCudaDeviceInfo();

} // namespace cuda
} // namespace mid360_bridge

#endif // MID360_TO_NVBLOX_BRIDGE__CUDA__BRIDGE_CONVERTER_GPU_HPP_
