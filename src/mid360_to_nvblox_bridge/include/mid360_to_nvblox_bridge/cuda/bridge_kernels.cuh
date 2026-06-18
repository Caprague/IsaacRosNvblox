// SPDX-License-Identifier: Apache-2.0
// CUDA Kernels for Mid360 Bridge - GPU Acceleration

#ifndef MID360_TO_NVBLOX_BRIDGE__CUDA__BRIDGE_KERNELS_CUH_
#define MID360_TO_NVBLOX_BRIDGE__CUDA__BRIDGE_KERNELS_CUH_

#include "mid360_to_nvblox_bridge/cuda/bridge_converter_gpu.hpp"
#include <cuda_runtime.h>
#include <device_launch_parameters.h>

namespace mid360_bridge {
namespace cuda {

// GPU grid cell
struct GridCellGPU {
  float depth;
  float depth_sum;
  int point_count;
  bool valid;
};

// ============================================================================
// Kernel 1: Cartesian to Spherical + Grid Mapping + Accumulation
// ============================================================================
__global__ void pointsToGridKernel(
    const float* __restrict__ points_x,
    const float* __restrict__ points_y,
    const float* __restrict__ points_z,
    const int num_points,
    const VirtualLidarConfig config,
    GridCellGPU* __restrict__ grid,
    int* __restrict__ valid_count)
{
  const int idx = blockIdx.x * blockDim.x + threadIdx.x;
  
  if (idx >= num_points) {
    return;
  }
  
  // Read point
  const float x = points_x[idx];
  const float y = points_y[idx];
  const float z = points_z[idx];
  
  // Skip NaN
  if (isnan(x) || isnan(y) || isnan(z)) {
    return;
  }
  
  // === Cartesian to Spherical (inline) ===
  const float range = sqrtf(x*x + y*y + z*z);
  
  // Range check
  if (range < config.min_range_m || range > config.max_range_m) {
    return;
  }
  
  // Use hardware SFU for trigonometric functions
  const float azimuth = atan2f(y, x);          // [-π, π]
  const float elevation = asinf(z / range);     // [-π/2, π/2]
  
  // === Spherical to Grid (inline) ===
  
  // Elevation bounds check
  if (elevation < config.min_elevation_rad || 
      elevation > config.max_elevation_rad) {
    return;
  }
  
  // Map azimuth to [0, width)
  float azimuth_norm = azimuth + M_PI;  // [0, 2π]
  int u_idx = __float2int_rn(azimuth_norm / config.azimuth_res_rad);
  
  // Handle wrap-around
  if (u_idx >= config.width) u_idx = config.width - 1;
  if (u_idx < 0) u_idx = 0;
  
  // Map elevation to [0, height)
  float elev_offset = elevation - config.min_elevation_rad;
  int v_idx = __float2int_rn(elev_offset / config.elevation_res_rad);
  
  // Clamp
  if (v_idx < 0) v_idx = 0;
  if (v_idx >= config.height) v_idx = config.height - 1;
  
  // === Atomic accumulation into grid ===
  const int grid_idx = v_idx * config.width + u_idx;
  
  // Use atomic operations for thread-safe accumulation
  atomicAdd(&grid[grid_idx].depth_sum, range);
  atomicAdd(&grid[grid_idx].point_count, 1);
  
  // Atomic increment of valid count (optional, for statistics)
  atomicAdd(valid_count, 1);
}

// ============================================================================
// Kernel 2: Aggregate grid cells (mean/min/max)
// ============================================================================
__global__ void aggregateGridKernel(
    GridCellGPU* __restrict__ grid,
    const int grid_size,
    const AggregationMethod method)
{
  const int idx = blockIdx.x * blockDim.x + threadIdx.x;
  
  if (idx >= grid_size) {
    return;
  }
  
  GridCellGPU& cell = grid[idx];
  
  if (cell.point_count > 0) {
    switch (method) {
      case AggregationMethod::MEAN:
        cell.depth = cell.depth_sum / static_cast<float>(cell.point_count);
        break;
      
      case AggregationMethod::MIN:
        // For min, depth_sum should already store min via atomicMin
        cell.depth = cell.depth_sum;
        break;
      
      case AggregationMethod::MAX:
        // For max, depth_sum should already store max via atomicMax
        cell.depth = cell.depth_sum;
        break;
    }
    cell.valid = true;
  } else {
    cell.valid = false;
    cell.depth = 0.0f;
  }
}

// ============================================================================
// Kernel 3: Fill holes via interpolation (single iteration)
// ============================================================================
__global__ void fillHolesKernel(
    const GridCellGPU* __restrict__ grid_in,
    GridCellGPU* __restrict__ grid_out,
    const int width,
    const int height)
{
  const int u = blockIdx.x * blockDim.x + threadIdx.x;
  const int v = blockIdx.y * blockDim.y + threadIdx.y;
  
  if (u >= width || v >= height) {
    return;
  }
  
  const int idx = v * width + u;
  
  // If already valid, just copy
  if (grid_in[idx].valid) {
    grid_out[idx] = grid_in[idx];
    return;
  }
  
  // Try to interpolate from neighbors
  float depth_sum = 0.0f;
  int neighbor_count = 0;
  
  // 4-connected neighbors
  // Left
  if (u > 0 && grid_in[v * width + (u-1)].valid) {
    depth_sum += grid_in[v * width + (u-1)].depth;
    neighbor_count++;
  }
  
  // Right
  if (u < width-1 && grid_in[v * width + (u+1)].valid) {
    depth_sum += grid_in[v * width + (u+1)].depth;
    neighbor_count++;
  }
  
  // Top
  if (v > 0 && grid_in[(v-1) * width + u].valid) {
    depth_sum += grid_in[(v-1) * width + u].depth;
    neighbor_count++;
  }
  
  // Bottom
  if (v < height-1 && grid_in[(v+1) * width + u].valid) {
    depth_sum += grid_in[(v+1) * width + u].depth;
    neighbor_count++;
  }
  
  // Horizontal wrap-around (azimuth is 360°)
  if (u == 0 && grid_in[v * width + (width-1)].valid) {
    depth_sum += grid_in[v * width + (width-1)].depth;
    neighbor_count++;
  }
  if (u == width-1 && grid_in[v * width + 0].valid) {
    depth_sum += grid_in[v * width + 0].depth;
    neighbor_count++;
  }
  
  // Need at least 2 neighbors for interpolation
  if (neighbor_count >= 2) {
    grid_out[idx].depth = depth_sum / static_cast<float>(neighbor_count);
    grid_out[idx].valid = true;
    grid_out[idx].point_count = 1;  // Mark as interpolated
  } else {
    grid_out[idx].valid = false;
    grid_out[idx].depth = 0.0f;
  }
}

// ============================================================================
// Kernel 4: Reconstruct pointcloud from grid
// ============================================================================
__global__ void reconstructPointcloudKernel(
    const GridCellGPU* __restrict__ grid,
    const VirtualLidarConfig config,
    float* __restrict__ points_x,
    float* __restrict__ points_y,
    float* __restrict__ points_z)
{
  const int u = blockIdx.x * blockDim.x + threadIdx.x;
  const int v = blockIdx.y * blockDim.y + threadIdx.y;
  
  if (u >= config.width || v >= config.height) {
    return;
  }
  
  const int idx = v * config.width + u;
  const GridCellGPU& cell = grid[idx];
  
  if (cell.valid) {
    // Grid indices back to spherical
    const float azimuth = -M_PI + (u + 0.5f) * config.azimuth_res_rad;
    const float elevation = config.min_elevation_rad + v * config.elevation_res_rad;
    const float range = cell.depth;
    
    // Spherical to Cartesian (use hardware SFU)
    const float cos_elev = cosf(elevation);
    points_x[idx] = range * cos_elev * cosf(azimuth);
    points_y[idx] = range * cos_elev * sinf(azimuth);
    points_z[idx] = range * sinf(elevation);
  } else {
    // Invalid point marked as NaN
    points_x[idx] = nanf("");
    points_y[idx] = nanf("");
    points_z[idx] = nanf("");
  }
}

// ============================================================================
// Kernel 5: Optimized version with min/max tracking
// ============================================================================
__global__ void pointsToGridMinMaxKernel(
    const float* __restrict__ points_x,
    const float* __restrict__ points_y,
    const float* __restrict__ points_z,
    const int num_points,
    const VirtualLidarConfig config,
    GridCellGPU* __restrict__ grid,
    const AggregationMethod method)
{
  const int idx = blockIdx.x * blockDim.x + threadIdx.x;
  
  if (idx >= num_points) {
    return;
  }
  
  const float x = points_x[idx];
  const float y = points_y[idx];
  const float z = points_z[idx];
  
  if (isnan(x) || isnan(y) || isnan(z)) {
    return;
  }
  
  // Cartesian to Spherical
  const float range = sqrtf(x*x + y*y + z*z);
  
  if (range < config.min_range_m || range > config.max_range_m) {
    return;
  }
  
  const float azimuth = atan2f(y, x);
  const float elevation = asinf(z / range);
  
  if (elevation < config.min_elevation_rad || 
      elevation > config.max_elevation_rad) {
    return;
  }
  
  // Spherical to Grid
  float azimuth_norm = azimuth + M_PI;
  int u_idx = __float2int_rn(azimuth_norm / config.azimuth_res_rad);
  if (u_idx >= config.width) u_idx = config.width - 1;
  if (u_idx < 0) u_idx = 0;
  
  float elev_offset = elevation - config.min_elevation_rad;
  int v_idx = __float2int_rn(elev_offset / config.elevation_res_rad);
  if (v_idx < 0) v_idx = 0;
  if (v_idx >= config.height) v_idx = config.height - 1;
  
  const int grid_idx = v_idx * config.width + u_idx;
  
  // Aggregation with atomic operations
  switch (method) {
    case AggregationMethod::MEAN:
      atomicAdd(&grid[grid_idx].depth_sum, range);
      break;
    
    case AggregationMethod::MIN:
      // Use atomic min (requires reinterpret cast for float)
      atomicMin((int*)&grid[grid_idx].depth_sum, __float_as_int(range));
      break;
    
    case AggregationMethod::MAX:
      // Use atomic max
      atomicMax((int*)&grid[grid_idx].depth_sum, __float_as_int(range));
      break;
  }
  
  atomicAdd(&grid[grid_idx].point_count, 1);
}

} // namespace cuda
} // namespace mid360_bridge

#endif  // MID360_TO_NVBLOX_BRIDGE__CUDA__BRIDGE_KERNELS_CUH_
