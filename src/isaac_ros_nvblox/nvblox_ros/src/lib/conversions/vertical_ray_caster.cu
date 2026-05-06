#include "nvblox_ros/conversions/vertical_ray_caster.hpp"
#include <nvblox/gpu_hash/internal/cuda/gpu_hash_interface.cuh>
#include <nvblox/gpu_hash/internal/cuda/gpu_indexing.cuh>

#include "nvblox/map/accessors.h"
#include "nvblox/map/blox.h"
#include "nvblox/map/common_names.h"
#include "nvblox/map/layer.h"
#include "nvblox/map/voxels.h"
#include "nvblox/gpu_hash/gpu_layer_view.h"
#include "nvblox/primitives/primitives.h"
#include "nvblox/primitives/scene.h"

#include <limits>
#include <rclcpp/rclcpp.hpp>

namespace nvblox {
namespace conversions {

/**
 * @brief 合并核函数：计算采样点坐标 + DDA体素步进光线投射 + 子体素线性插值
 *
 * 改进点：
 * 1. 合并原 computeSamplePointsKernel 与 rayCastingVoxelsOnGPU，消除中间全局内存读写
 * 2. 按体素索引逐层步进（z-1），仅在跨越 block 边界时查询 hash map，缓存 block 指针
 * 3. 利用 TSDF distance 符号变化做线性插值，子体素精度定位表面（替代手动微调）
 */
__global__ void sampleTerrainKernel(
    Index3DDeviceHashMapType<TsdfBlock> block_hash,
    Vector3f* d_terrain_points,
    bool* d_point_validity,
    const float confidence_weight_threshold,
    const float block_size,
    const int num_points,
    const float max_distance,
    const int x_steps,
    const int y_steps,
    const float range_x,
    const float range_y,
    const float resolution,
    const float robot_pos_x,
    const float robot_pos_y,
    const float robot_pos_z,
    const float yaw_rot_00,
    const float yaw_rot_01,
    const float yaw_rot_10,
    const float yaw_rot_11,
    const float x_offset,
    const float y_offset,
    const float z_offset)
{
  const int idx = threadIdx.x + blockIdx.x * blockDim.x;
  if (idx >= num_points) {
    return;
  }

  // ---- 1. 合并：计算采样点坐标 ----
  const int i = idx / x_steps;
  const int j = idx % x_steps;

  const float rel_x = -range_x * 0.5f + j * resolution;
  const float rel_y = -range_y * 0.5f + i * resolution;
  const float local_x = rel_x + x_offset;
  const float local_y = rel_y + y_offset;

  const float global_x = robot_pos_x + yaw_rot_00 * local_x + yaw_rot_01 * local_y;
  const float global_y = robot_pos_y + yaw_rot_10 * local_x + yaw_rot_11 * local_y;
  const float global_z = robot_pos_z + z_offset;

  const Vector3f beg_position(global_x, global_y, global_z);

  // ---- 2. DDA式体素步进（垂直向下） ----
  Vector3f end_position = beg_position;
  end_position.z() = beg_position.z() - max_distance;

  Index3D beg_block_idx, beg_voxel_idx;
  Index3D end_block_idx, end_voxel_idx;
  getBlockAndVoxelIndexFromPositionInLayer(block_size, beg_position,
                                           &beg_block_idx, &beg_voxel_idx);
  getBlockAndVoxelIndexFromPositionInLayer(block_size, end_position,
                                           &end_block_idx, &end_voxel_idx);

  // z方向总步数（体素级别，包含两端）
  constexpr int kVoxelsPerSide = TsdfBlock::kVoxelsPerSide;
  int z_steps = (beg_block_idx.z() - end_block_idx.z()) * kVoxelsPerSide
              + (beg_voxel_idx.z() - end_voxel_idx.z());
  if (z_steps < 0) {
    z_steps = 0;
  }

  // 缓存当前 block 指针，仅在 block_idx 变化时查询 hash map
  TsdfBlock* cached_block = nullptr;
  Index3D cached_block_idx(-1, -1, -1);

  // 记录上一个 distance > 0 的体素，用于子体素插值
  bool found_positive = false;
  float last_positive_distance = 0.0f;
  Vector3f last_positive_position;

  bool has_valid_voxel = false;
  Vector3f valid_terrain_point;

  Index3D block_idx = beg_block_idx;
  Index3D voxel_idx = beg_voxel_idx;

  for (int step = 0; step <= z_steps; ++step) {
    // block 变化时更新缓存
    if (cached_block == nullptr || block_idx != cached_block_idx) {
      auto it = block_hash.find(block_idx);
      if (it != block_hash.end()) {
        cached_block = it->second;
        cached_block_idx = block_idx;
      } else {
        cached_block = nullptr;
      }
    }

    if (cached_block != nullptr) {
      const TsdfVoxel& voxel =
          cached_block->voxels[voxel_idx.x()][voxel_idx.y()][voxel_idx.z()];

      if (voxel.weight > confidence_weight_threshold) {
        const Vector3f current_pos =
            getCenterPositionFromBlockIndexAndVoxelIndex(block_size, block_idx,
                                                         voxel_idx);

        if (voxel.distance > 0.0f) {
          // 记录正距离体素
          found_positive = true;
          last_positive_distance = voxel.distance;
          last_positive_position = current_pos;
        } else {
          // 找到 occupied 体素 (distance <= 0)
          if (found_positive) {
            // 子体素线性插值：在 last_positive (dist>0) 与 current (dist<0) 之间
            const float dist_above = last_positive_distance;
            const float dist_below = fabsf(voxel.distance);
            const float t = dist_above / (dist_above + dist_below);
            const float z_cross =
                last_positive_position.z() +
                t * (current_pos.z() - last_positive_position.z());
            valid_terrain_point =
                Vector3f(current_pos.x(), current_pos.y(), z_cross);
          } else {
            // 未遇到 positive 体素即进入 negative，直接返回当前体素中心
            valid_terrain_point = current_pos;
          }
          has_valid_voxel = true;
          break;
        }
      }
    }

    // 步进到下一个 voxel（z - 1 方向）
    voxel_idx.z()--;
    if (voxel_idx.z() < 0) {
      voxel_idx.z() = kVoxelsPerSide - 1;
      block_idx.z()--;
    }
  }

  d_point_validity[idx] = has_valid_voxel;
  if (has_valid_voxel) {
    d_terrain_points[idx] = valid_terrain_point;
  } else {
    d_terrain_points[idx] = Vector3f(beg_position.x(), beg_position.y(), 0.0f);
  }
}

// ---- Block指针共享内存缓存（Optimization 2）----
struct BlockCacheEntry {
  int16_t gx, gy, gz;     // 相对网格坐标
  const TsdfBlock* ptr;   // Block指针（只读）
};
constexpr int SHARED_CACHE_SIZE = 32;  // 32 * ~16B = ~512B shared memory

/**
 * @brief 优化核函数：预取Block指针网格 + 共享内存缓存 + DDA步进 + 子体素插值
 *
 * 优化点（在 sampleTerrainKernel 基础上）：
 * 1. CPU端预取Block指针到密集3D网格，kernel内O(1)直接索引，消除GPU hash map查找
 * 2. 共享内存缓存：同一线程块内的相邻采样点共享Block指针，减少全局内存读取
 * 3. 三级查找：寄存器缓存(主) → 共享内存缓存(辅) → 全局网格(兜底)
 *
 * 保留：DDA体素索引步进 + 子体素线性插值（与sampleTerrainKernel一致）
 */
__global__ void sampleTerrainKernelOptimized(
    const TsdfBlock* const* block_ptr_grid,  // 预取的Block指针3D网格（展平）
    const Index3D min_block_idx,        // 网格原点（block坐标）
    const Index3D grid_dims,            // 网格维度
    Vector3f* d_terrain_points,
    bool* d_point_validity,
    const float confidence_weight_threshold,
    const float block_size,
    const int num_points,
    const float max_distance,
    const int x_steps,
    const int y_steps,
    const float range_x,
    const float range_y,
    const float resolution,
    const float robot_pos_x,
    const float robot_pos_y,
    const float robot_pos_z,
    const float yaw_rot_00,
    const float yaw_rot_01,
    const float yaw_rot_10,
    const float yaw_rot_11,
    const float x_offset,
    const float y_offset,
    const float z_offset)
{
  // ---- 共享内存Block指针缓存初始化 ----
  __shared__ BlockCacheEntry s_cache[SHARED_CACHE_SIZE];
  __shared__ int s_cache_count;
  if (threadIdx.x == 0) {
    s_cache_count = 0;
  }
  __syncthreads();

  const int idx = threadIdx.x + blockIdx.x * blockDim.x;
  if (idx >= num_points) {
    return;
  }

  // ---- 1. 计算采样点坐标 ----
  const int i = idx / x_steps;
  const int j = idx % x_steps;

  const float rel_x = -range_x * 0.5f + j * resolution;
  const float rel_y = -range_y * 0.5f + i * resolution;
  const float local_x = rel_x + x_offset;
  const float local_y = rel_y + y_offset;

  const float global_x = robot_pos_x + yaw_rot_00 * local_x + yaw_rot_01 * local_y;
  const float global_y = robot_pos_y + yaw_rot_10 * local_x + yaw_rot_11 * local_y;
  const float global_z = robot_pos_z + z_offset;

  const Vector3f beg_position(global_x, global_y, global_z);

  // ---- 2. DDA式体素步进（垂直向下） ----
  Vector3f end_position = beg_position;
  end_position.z() = beg_position.z() - max_distance;

  Index3D beg_block_idx, beg_voxel_idx;
  Index3D end_block_idx, end_voxel_idx;
  getBlockAndVoxelIndexFromPositionInLayer(block_size, beg_position,
                                           &beg_block_idx, &beg_voxel_idx);
  getBlockAndVoxelIndexFromPositionInLayer(block_size, end_position,
                                           &end_block_idx, &end_voxel_idx);

  constexpr int kVoxelsPerSide = TsdfBlock::kVoxelsPerSide;
  int z_steps = (beg_block_idx.z() - end_block_idx.z()) * kVoxelsPerSide
              + (beg_voxel_idx.z() - end_voxel_idx.z());
  if (z_steps < 0) {
    z_steps = 0;
  }

  // 寄存器级缓存（主缓存：同Block内Z步进复用）
  const TsdfBlock* cached_block = nullptr;
  Index3D cached_block_idx(-1, -1, -1);

  bool found_positive = false;
  float last_positive_distance = 0.0f;
  Vector3f last_positive_position;

  bool has_valid_voxel = false;
  Vector3f valid_terrain_point;

  Index3D block_idx = beg_block_idx;
  Index3D voxel_idx = beg_voxel_idx;

  for (int step = 0; step <= z_steps; ++step) {
    // ---- 三级Block指针查找 ----
    if (cached_block == nullptr || block_idx != cached_block_idx) {
      // 计算相对网格坐标
      const int gx = block_idx.x() - min_block_idx.x();
      const int gy = block_idx.y() - min_block_idx.y();
      const int gz = block_idx.z() - min_block_idx.z();

      // 第2级：扫描共享内存缓存
      cached_block = nullptr;
      const int scan_count = min(s_cache_count, SHARED_CACHE_SIZE);
      for (int c = 0; c < scan_count; ++c) {
        if (s_cache[c].gx == static_cast<int16_t>(gx) &&
            s_cache[c].gy == static_cast<int16_t>(gy) &&
            s_cache[c].gz == static_cast<int16_t>(gz)) {
          cached_block = s_cache[c].ptr;
          break;
        }
      }

      // 第3级：全局网格直接索引（O(1)，无hash）
      if (cached_block == nullptr) {
        if (gx >= 0 && gx < grid_dims.x() &&
            gy >= 0 && gy < grid_dims.y() &&
            gz >= 0 && gz < grid_dims.z()) {
          cached_block = block_ptr_grid[gz * grid_dims.y() * grid_dims.x()
                                       + gy * grid_dims.x() + gx];
        }
        // 尽力插入共享缓存（atomicAdd分配槽位，跨warp尽力共享）
        int slot = atomicAdd(&s_cache_count, 1);
        if (slot < SHARED_CACHE_SIZE) {
          // 先写ptr再写坐标，防止其他线程读到坐标匹配但ptr未写入的半成品条目
          s_cache[slot].ptr = cached_block;
          __threadfence_block();
          s_cache[slot].gx = static_cast<int16_t>(gx);
          s_cache[slot].gy = static_cast<int16_t>(gy);
          s_cache[slot].gz = static_cast<int16_t>(gz);
        }
      }

      cached_block_idx = block_idx;
    }

    // ---- 体素访问与表面检测（与sampleTerrainKernel一致） ----
    if (cached_block != nullptr) {
      const TsdfVoxel& voxel =
          cached_block->voxels[voxel_idx.x()][voxel_idx.y()][voxel_idx.z()];

      if (voxel.weight > confidence_weight_threshold) {
        const Vector3f current_pos =
            getCenterPositionFromBlockIndexAndVoxelIndex(block_size, block_idx,
                                                         voxel_idx);

        if (voxel.distance > 0.0f) {
          found_positive = true;
          last_positive_distance = voxel.distance;
          last_positive_position = current_pos;
        } else {
          if (found_positive) {
            // 子体素线性插值
            const float dist_above = last_positive_distance;
            const float dist_below = fabsf(voxel.distance);
            const float t = dist_above / (dist_above + dist_below);
            const float z_cross =
                last_positive_position.z() +
                t * (current_pos.z() - last_positive_position.z());
            valid_terrain_point =
                Vector3f(current_pos.x(), current_pos.y(), z_cross);
          } else {
            valid_terrain_point = current_pos;
          }
          has_valid_voxel = true;
          break;
        }
      }
    }

    // 步进到下一个 voxel（z - 1 方向）
    voxel_idx.z()--;
    if (voxel_idx.z() < 0) {
      voxel_idx.z() = kVoxelsPerSide - 1;
      block_idx.z()--;
    }
  }

  d_point_validity[idx] = has_valid_voxel;
  if (has_valid_voxel) {
    d_terrain_points[idx] = valid_terrain_point;
  } else {
    d_terrain_points[idx] = Vector3f(beg_position.x(), beg_position.y(), 0.0f);
  }
}

/**
 * @brief GPU无效高程点填充：单步迭代传播核函数
 *
 * 每个线程处理一个网格点：
 * - 有效点：直接传递有效性标记
 * - 无效点：从8邻域有效点按反距离加权(IDW)插值z值，标记为已填充
 * - 无有效邻域的无效点：保持无效，等待下一轮传播
 */
__global__ void propagateElevationKernel(
    Vector3f* d_terrain_points,
    const bool* d_valid_in,
    bool* d_valid_out,
    const int x_steps,
    const int y_steps)
{
  const int idx = threadIdx.x + blockIdx.x * blockDim.x;
  const int total = x_steps * y_steps;
  if (idx >= total) return;

  const int row = idx / x_steps;
  const int col = idx % x_steps;

  // 有效点：直接传递
  if (d_valid_in[idx]) {
    d_valid_out[idx] = true;
    return;
  }

  // 无效点：从8邻域有效点进行IDW插值
  float weight_sum = 0.0f;
  float z_weighted = 0.0f;

  for (int dr = -1; dr <= 1; ++dr) {
    for (int dc = -1; dc <= 1; ++dc) {
      if (dr == 0 && dc == 0) continue;
      const int nr = row + dr;
      const int nc = col + dc;
      if (nr < 0 || nr >= y_steps || nc < 0 || nc >= x_steps) continue;
      const int nidx = nr * x_steps + nc;
      if (d_valid_in[nidx]) {
        const float dist_sq = static_cast<float>(dr * dr + dc * dc);
        const float w = 1.0f / dist_sq;  // IDW power=2
        weight_sum += w;
        z_weighted += w * d_terrain_points[nidx].z();
      }
    }
  }

  if (weight_sum > 0.0f) {
    Vector3f pt = d_terrain_points[idx];
    pt.z() = z_weighted / weight_sum;
    d_terrain_points[idx] = pt;
    d_valid_out[idx] = true;
  } else {
    d_valid_out[idx] = false;
  }
}

/**
 * @brief 3x3高斯平滑核函数（对有效点的高程z值进行平滑）
 *
 * 权重布局（近似高斯，总和=16便于整数运算）：
 *   1  2  1
 *   2  4  2
 *   1  2  1
 *
 * 仅对有效点平滑，无效点保持不变。边界点仅使用网格内有效邻域。
 */
__global__ void smoothElevationKernel(
    const Vector3f* d_terrain_points_in,
    Vector3f* d_terrain_points_out,
    const bool* d_valid,
    const int x_steps,
    const int y_steps)
{
  const int idx = threadIdx.x + blockIdx.x * blockDim.x;
  const int total = x_steps * y_steps;
  if (idx >= total) return;

  // 无效点直接拷贝
  if (!d_valid[idx]) {
    d_terrain_points_out[idx] = d_terrain_points_in[idx];
    return;
  }

  const int row = idx / x_steps;
  const int col = idx % x_steps;

  // 3x3高斯加权平滑
  constexpr int kWeight[3][3] = {{1, 2, 1}, {2, 4, 2}, {1, 2, 1}};
  float z_sum = 0.0f;
  float w_sum = 0.0f;

  for (int dr = -1; dr <= 1; ++dr) {
    for (int dc = -1; dc <= 1; ++dc) {
      const int nr = row + dr;
      const int nc = col + dc;
      if (nr < 0 || nr >= y_steps || nc < 0 || nc >= x_steps) continue;
      const int nidx = nr * x_steps + nc;
      if (d_valid[nidx]) {
        const float w = static_cast<float>(kWeight[dr + 1][dc + 1]);
        z_sum += w * d_terrain_points_in[nidx].z();
        w_sum += w;
      }
    }
  }

  Vector3f pt = d_terrain_points_in[idx];
  if (w_sum > 0.0f) {
    pt.z() = z_sum / w_sum;
  }
  d_terrain_points_out[idx] = pt;
}

void CudaVerticalRayCaster::fillInvalidElevationGPU(
    device_vector<Vector3f>& d_terrain_points,
    device_vector<bool>& d_point_validity,
    int x_steps, int y_steps)
{
  const int total = x_steps * y_steps;
  if (total == 0) return;

  const int threads_per_block = 256;
  const int num_blocks = (total + threads_per_block - 1) / threads_per_block;
  const int max_iters = std::max(x_steps, y_steps);

  // 确保双缓冲区已分配
  d_valid_buf_.resizeAsync(total, height_scan_stream_);
  d_smooth_buf_.resizeAsync(total, height_scan_stream_);

  bool* valid_in = d_point_validity.data();
  bool* valid_out = d_valid_buf_.data();

  // 迭代传播：每轮将有效区域向外扩展1格
  for (int iter = 0; iter < max_iters; ++iter) {
    propagateElevationKernel<<<num_blocks, threads_per_block, 0, height_scan_stream_>>>(
        d_terrain_points.data(), valid_in, valid_out, x_steps, y_steps);
    std::swap(valid_in, valid_out);
  }

  // 确保最终有效性标记回到d_point_validity
  if (valid_in == d_valid_buf_.data()) {
    cudaMemcpyAsync(d_point_validity.data(), d_valid_buf_.data(),
                    sizeof(bool) * total, cudaMemcpyDeviceToDevice,
                    static_cast<cudaStream_t>(height_scan_stream_));
  }

  // 3x3高斯平滑（2轮），消除填充边界跳变
  constexpr int kSmoothRounds = 2;
  const bool* smooth_valid = d_point_validity.data();
  for (int round = 0; round < kSmoothRounds; ++round) {
    smoothElevationKernel<<<num_blocks, threads_per_block, 0, height_scan_stream_>>>(
        d_terrain_points.data(), d_smooth_buf_.data(),
        smooth_valid, x_steps, y_steps);
    std::swap(d_terrain_points, d_smooth_buf_);
  }
  // 确保最终结果在d_terrain_points中
  // 经过偶数轮swap后，d_terrain_points指向原始缓冲区
}

CudaVerticalRayCaster::CudaVerticalRayCaster(float threshold)
    : confidence_weight_threshold_(threshold)
{}

CudaVerticalRayCaster::~CudaVerticalRayCaster()
{}

void CudaVerticalRayCaster::setConfidenceWeightThreshold(float threshold)
{
  confidence_weight_threshold_ = threshold;
}

bool CudaVerticalRayCaster::sampleTerrainPoints(
  std::vector<float>& sample_points_x,
  std::vector<float>& sample_points_y,
  std::vector<float>& sample_points_z,
  TsdfLayer& tsdf_layer,
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
)
{
  // 更新配置参数
  block_size_ = tsdf_layer.block_size();
  total_points_ = x_steps * y_steps;
  max_distance_ = max_casting_depth;
  
  // 增加采样计数器
  sample_count_++;
  
  // 调整容器大小
  sample_points_x.clear();
  sample_points_y.clear();
  sample_points_z.clear();
  sample_points_x.resize(total_points_, 0.0f);
  sample_points_y.resize(total_points_, 0.0f);
  sample_points_z.resize(total_points_, 0.0f);

  // 获取机器人位置和旋转
  Vector3f robot_position = robot_pose.translation();
  const auto& rotation = robot_pose.rotation();
  float yaw = std::atan2(rotation(1, 0), rotation(0, 0));
  float yaw_rot_00 = cos(yaw);
  float yaw_rot_01 = -sin(yaw);
  float yaw_rot_10 = sin(yaw);
  float yaw_rot_11 = cos(yaw);

  // GPU Output space
  device_vector<Vector3f> d_terrain_points_(total_points_);
  device_vector<bool> d_point_validity_(total_points_);
  // 使用独立非阻塞流初始化设备内存，避免与主映射流阻塞
  d_terrain_points_.setZeroAsync(height_scan_stream_);
  d_point_validity_.setZeroAsync(height_scan_stream_);

  // 配置CUDA内核参数
  const int threads_per_block = 256;
  const int num_blocks_needed = (total_points_ + threads_per_block - 1) / threads_per_block;

  // ---- AABB计算：确定采样体积的Block索引范围 ----
  // 4个采样矩形角点（本地坐标系）
  const float corners_local_x[4] = {
    -range_x * 0.5f + x_offset,   // 左下
     range_x * 0.5f + x_offset,   // 右下
     range_x * 0.5f + x_offset,   // 右上
    -range_x * 0.5f + x_offset,   // 左上
  };
  const float corners_local_y[4] = {
    -range_y * 0.5f + y_offset,
    -range_y * 0.5f + y_offset,
     range_y * 0.5f + y_offset,
     range_y * 0.5f + y_offset,
  };

  // 旋转到世界坐标系，求XY范围
  float world_min_x = std::numeric_limits<float>::max();
  float world_max_x = std::numeric_limits<float>::lowest();
  float world_min_y = std::numeric_limits<float>::max();
  float world_max_y = std::numeric_limits<float>::lowest();

  for (int c = 0; c < 4; ++c) {
    const float wx = robot_position.x() + yaw_rot_00 * corners_local_x[c] + yaw_rot_01 * corners_local_y[c];
    const float wy = robot_position.y() + yaw_rot_10 * corners_local_x[c] + yaw_rot_11 * corners_local_y[c];
    world_min_x = std::min(world_min_x, wx);
    world_max_x = std::max(world_max_x, wx);
    world_min_y = std::min(world_min_y, wy);
    world_max_y = std::max(world_max_y, wy);
  }

  // Z范围：从光线起点到终点
  const float world_max_z = robot_position.z() + z_offset;
  const float world_min_z = world_max_z - max_distance_;

  // 世界AABB → Block索引范围
  const Index3D min_block_idx = getBlockIndexFromPositionInLayer(
      block_size_, Vector3f(world_min_x, world_min_y, world_min_z));
  const Index3D max_block_idx = getBlockIndexFromPositionInLayer(
      block_size_, Vector3f(world_max_x, world_max_y, world_max_z));

  grid_dims_ = max_block_idx - min_block_idx + Index3D(1, 1, 1);
  const int grid_total = grid_dims_.x() * grid_dims_.y() * grid_dims_.z();

  // ---- 判断是否使用优化路径（Block指针预取网格） ----
  bool use_grid_path = (grid_total > 0 && grid_total <= kMaxGridTotal);

  if (!use_grid_path) {
    RCLCPP_WARN(rclcpp::get_logger("CudaVerticalRayCaster"),
                "\n\n[HeightScan] Block grid total %d exceeds cap %d, falling back to hash map path\n",
                grid_total, kMaxGridTotal);
  }

  if (use_grid_path) {
    // ---- 构建Block指针预取网格 ----
    h_block_ptr_grid_.resize(grid_total, nullptr);

    for (int gz = 0; gz < grid_dims_.z(); ++gz) {
      for (int gy = 0; gy < grid_dims_.y(); ++gy) {
        for (int gx = 0; gx < grid_dims_.x(); ++gx) {
          const Index3D block_idx = min_block_idx + Index3D(gx, gy, gz);
          const auto block_ptr = tsdf_layer.getBlockAtIndex(block_idx);
          h_block_ptr_grid_[gz * grid_dims_.y() * grid_dims_.x()
                           + gy * grid_dims_.x() + gx] =
              block_ptr.get();  // nullptr if block not allocated
        }
      }
    }

    // 异步复制到GPU（在height_scan_stream_上执行）
    d_block_ptr_grid_.copyFromAsync(h_block_ptr_grid_, height_scan_stream_);

    // 启动优化核函数：预取网格 + 共享内存缓存 + DDA步进 + 子体素插值
    sampleTerrainKernelOptimized<<<num_blocks_needed, threads_per_block, 0, height_scan_stream_>>>(
        d_block_ptr_grid_.data(),
        min_block_idx,
        grid_dims_,
        d_terrain_points_.data(),
        d_point_validity_.data(),
        confidence_weight_threshold_,
        block_size_,
        total_points_,
        max_distance_,
        x_steps,
        y_steps,
        range_x,
        range_y,
        grid_resolution,
        robot_position.x(),
        robot_position.y(),
        robot_position.z(),
        yaw_rot_00,
        yaw_rot_01,
        yaw_rot_10,
        yaw_rot_11,
        x_offset,
        y_offset,
        z_offset);
  } else {
    // ---- 回退路径：使用GPU hash map ----
    // 每次都重新获取GPU层视图，避免缓存过期引用导致的内存问题
    // getGpuLayerView()在主CUDA流上执行，确保GPU哈希表与最新TSDF层数据同步
    const GPULayerView<TsdfBlock>& gpu_layer_view = tsdf_layer.getGpuLayerView(cuda_stream);

    // 启动原始核函数：hash map查找 + DDA步进 + 子体素插值
    sampleTerrainKernel<<<num_blocks_needed, threads_per_block, 0, height_scan_stream_>>>(
        gpu_layer_view.getHash().impl_,
        d_terrain_points_.data(),
        d_point_validity_.data(),
        confidence_weight_threshold_,
        block_size_,
        total_points_,
        max_distance_,
        x_steps,
        y_steps,
        range_x,
        range_y,
        grid_resolution,
        robot_position.x(),
        robot_position.y(),
        robot_position.z(),
        yaw_rot_00,
        yaw_rot_01,
        yaw_rot_10,
        yaw_rot_11,
        x_offset,
        y_offset,
        z_offset);
  }

  cudaError_t err = cudaPeekAtLastError();
  if (err != cudaSuccess) {
    RCLCPP_ERROR(rclcpp::get_logger("CudaVerticalRayCaster"),
                 "sampleTerrainKernel%s failed: %s",
                 use_grid_path ? "Optimized" : "",
                 cudaGetErrorString(err));
    return false;
  }

  // GPU加速：填充无效高程点（IDW插值 + 迭代传播，在height_scan_stream_上异步执行）
  fillInvalidElevationGPU(d_terrain_points_, d_point_validity_, x_steps, y_steps);

  // GPU -> CPU 数据传输（在独立流上异步执行）
  h_terrain_points_.copyFromAsync(d_terrain_points_, height_scan_stream_);
  h_point_validity_.copyFromAsync(d_point_validity_, height_scan_stream_);
  
  // 仅同步独立流，确保光线追踪和D2H传输完成后再访问数据
  // 不再同步主映射流，避免阻塞integrateDepth/updateEsdf等操作
  cudaError_t sync_err = cudaStreamSynchronize(height_scan_stream_);
  if (sync_err != cudaSuccess) {
    RCLCPP_ERROR(rclcpp::get_logger("CudaVerticalRayCaster"), 
                 "Height scan stream synchronization failed: %s", cudaGetErrorString(sync_err));
    return false;
  }
  
  // 将结果转换为输出格式（此时数据已经完全传输完成）
  for (int i = 0; i < total_points_; ++i)
  {
    sample_points_x[i] = h_terrain_points_[i].x();
    sample_points_y[i] = h_terrain_points_[i].y();
    sample_points_z[i] = h_terrain_points_[i].z();
  }

  return true;
}

bool CudaVerticalRayCaster::initializeGroundPlane(TsdfLayer& tsdf_layer,
                                                 const Transform& robot_pose,
                                                 const CudaStream& cuda_stream) {
  // 获取机器人位置
  Vector3f robot_position = robot_pose.translation();
  
  // 计算地平面高度（使用参数化的高度偏移量）
  const float ground_height = robot_position.z() + ground_plane_height_offset_;
  const float voxel_size = tsdf_layer.voxel_size();
  const float half_size = 1.0f;  // 2m x 2m，半边长为1m
  
  // 创建 CPU 层来生成地平面
  TsdfLayer tsdf_layer_host(voxel_size, MemoryType::kHost);
  
  // 创建场景并添加地平面
  primitives::Scene ground_scene;
  
  // 设置场景边界
  ground_scene.aabb() = AxisAlignedBoundingBox(
    Vector3f(robot_position.x() - half_size, robot_position.y() - half_size, ground_height - voxel_size),
    Vector3f(robot_position.x() + half_size, robot_position.y() + half_size, ground_height + voxel_size)
  );
  
  // 添加地平面
  ground_scene.addGroundLevel(ground_height);
  
  // 添加边界限制范围
  ground_scene.addPlaneBoundaries(
    robot_position.x() - half_size, robot_position.x() + half_size,
    robot_position.y() - half_size, robot_position.y() + half_size
  );
  
  // 在 CPU 层上生成地平面
  ground_scene.generateLayerFromScene<TsdfVoxel>(max_distance_, &tsdf_layer_host);
  
  // 获取 CPU 层中所有已分配的块索引
  std::vector<Index3D> cpu_block_indices = tsdf_layer_host.getAllBlockIndices();
  
  // 提高地平面体素的置信度权重，确保光线投射能够检测到
  // generateLayerFromScene 默认设置 weight = 1.0f，但置信度阈值通常是 2.5f
  const float ground_plane_weight = 5.0f;  // 高于置信度阈值
  for (const Index3D& block_idx : cpu_block_indices) {
    TsdfBlock::Ptr cpu_block = tsdf_layer_host.getBlockAtIndex(block_idx);
    if (cpu_block != nullptr) {
      // 遍历块中的所有体素，提高权重
      for (int z = 0; z < TsdfBlock::kVoxelsPerSide; ++z) {
        for (int y = 0; y < TsdfBlock::kVoxelsPerSide; ++y) {
          for (int x = 0; x < TsdfBlock::kVoxelsPerSide; ++x) {
            TsdfVoxel& voxel = cpu_block->voxels[z][y][x];
            // 只提高占据体素的权重（distance < 0）
            if (voxel.distance < 0.0f) {
              voxel.weight = ground_plane_weight;
            }
          }
        }
      }
    }
  }
  
  // 在 GPU 层中分配这些块
  if (!cpu_block_indices.empty()) {
    tsdf_layer.allocateBlocksAtIndices(cpu_block_indices, cuda_stream);
  }
  
  // 同步CUDA流，确保块分配完成
  cudaStreamSynchronize(cuda_stream);
  
  // 逐个复制块数据
  for (const Index3D& block_idx : cpu_block_indices) {
    TsdfBlock::Ptr cpu_block = tsdf_layer_host.getBlockAtIndex(block_idx);
    TsdfBlock::Ptr gpu_block = tsdf_layer.getBlockAtIndex(block_idx);
    
    if (cpu_block != nullptr && gpu_block != nullptr) {
      // 复制块数据从 CPU 到 GPU
      cudaMemcpyAsync(gpu_block.get(), cpu_block.get(), sizeof(TsdfBlock), 
                     cudaMemcpyHostToDevice, cuda_stream);
    }
  }
  
  // 同步CUDA流，确保所有复制完成
  cudaStreamSynchronize(cuda_stream);
  
  // 设置标志位
  ground_plane_initialized_ = true;
  
  RCLCPP_INFO(rclcpp::get_logger("CudaVerticalRayCaster"),
              "Ground plane initialized at height %.3f m (offset=%.3f m, after %d samples, delay=%d)",
              ground_height, ground_plane_height_offset_, sample_count_, ground_plane_init_delay_);
  
  return true;
}

}  // namespace conversions
}  // namespace nvblox
