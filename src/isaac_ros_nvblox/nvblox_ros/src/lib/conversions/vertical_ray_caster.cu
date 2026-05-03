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

#include <queue>
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

CudaVerticalRayCaster::CudaVerticalRayCaster(float threshold)
    : confidence_weight_threshold_(threshold)
{}

CudaVerticalRayCaster::~CudaVerticalRayCaster()
{}

void CudaVerticalRayCaster::setConfidenceWeightThreshold(float threshold)
{
  confidence_weight_threshold_ = threshold;
}

// 改进的填充无效高程点函数 - 支持深层无效区域填充
void CudaVerticalRayCaster::fillInvalidElevationPoints(std::vector<float>& elevation_map, 
                                                     const host_vector<bool>& valid_mask,
                                                     int x_steps, int y_steps) {
    const int total_size = x_steps * y_steps;
    
    // 检查是否存在无效点
    bool has_invalid = false;
    for (int i = 0; i < total_size; ++i) {
        if (!valid_mask[i]) {
            has_invalid = true;
            break;
        }
    }
    
    // 如果没有无效点，直接返回
    if (!has_invalid) {
        return;
    }
    
    // 定义相邻点的偏移量（8邻域，提供更多填充方向）
    constexpr std::array<std::pair<int, int>, 8> neighbors = {
        {{-1, -1}, {-1, 0}, {-1, 1},
         {0, -1},          {0, 1},
         {1, -1},  {1, 0},  {1, 1}}
    };
    
    // 创建标记数组，记录哪些点已经被填充
    std::vector<bool> filled_mask(total_size, false);
    
    // 第一轮：标记所有初始有效点
    std::queue<std::pair<int, int>> frontier;
    for (int i = 0; i < y_steps; ++i) {
        for (int j = 0; j < x_steps; ++j) {
            const int idx = i * x_steps + j;
            if (valid_mask[idx]) {
                frontier.emplace(i, j);
                filled_mask[idx] = true;
            }
        }
    }
    
    // 如果没有任何有效点，使用默认值填充所有点并返回
    if (frontier.empty()) {
      const float default_value = -0.2f;
      std::fill(elevation_map.begin(), elevation_map.end(), default_value);
      return;
    }
    
    // 多轮洪水填充，直到所有点都被填充
    int iteration = 0;
    const int max_iterations = std::max(x_steps, y_steps) * 2; // 足够覆盖整个网格
    
    while (!frontier.empty() && iteration < max_iterations) {
        const int current_level_size = frontier.size();
        std::queue<std::pair<int, int>> next_frontier;
        
        // 处理当前层级的所有点
        for (int k = 0; k < current_level_size; ++k) {
            auto [i, j] = frontier.front();
            frontier.pop();
            
            const int current_idx = i * x_steps + j;
            
            // 填充当前点的所有无效邻居
            for (const auto& [di, dj] : neighbors) {
                int ni = i + di;
                int nj = j + dj;
                
                // 检查边界
                if (ni >= 0 && ni < y_steps && nj >= 0 && nj < x_steps) {
                    const int neighbor_idx = ni * x_steps + nj;
                    
                    // 如果邻居点无效且尚未被填充
                    if (!valid_mask[neighbor_idx] && !filled_mask[neighbor_idx]) {
                        // 使用当前有效点的值填充邻居
                        elevation_map[neighbor_idx] = elevation_map[current_idx];
                        filled_mask[neighbor_idx] = true;
                        next_frontier.emplace(ni, nj);
                    }
                }
            }
        }
        
        // 准备下一轮迭代
        frontier = std::move(next_frontier);
        iteration++;
        
        // 检查是否所有点都已被填充
        bool all_filled = true;
        for (int i = 0; i < total_size && all_filled; ++i) {
            if (!valid_mask[i] && !filled_mask[i]) {
                all_filled = false;
            }
        }
        
        if (all_filled) {
            break;
        }
    }
    
    // // 最终检查：确保所有无效点都被填充（处理极端情况）
    // for (int i = 0; i < total_size; ++i) {
    //     if (!valid_mask[i] && !filled_mask[i]) {
    //         // 查找最近的有效点（使用曼哈顿距离）
    //         float nearest_value = -0.22f; // 默认值
    //         int min_distance = std::max(x_steps, y_steps) + 1;
            
    //         for (int j = 0; j < total_size; ++j) {
    //             if (valid_mask[j] || filled_mask[j]) {
    //                 int i_row = i / x_steps, i_col = i % x_steps;
    //                 int j_row = j / x_steps, j_col = j % x_steps;
    //                 int distance = std::abs(i_row - j_row) + std::abs(i_col - j_col);
                    
    //                 if (distance < min_distance) {
    //                     min_distance = distance;
    //                     nearest_value = elevation_map[j];
    //                 }
    //             }
    //         }
            
    //         elevation_map[i] = nearest_value;
    //         filled_mask[i] = true;
    //     }
    // }
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
  voxel_size_ = tsdf_layer.voxel_size();
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
  
  // 每次都重新获取GPU层视图，避免缓存过期引用导致的内存问题
  // 注意：getGpuLayerView() 会更新 GPU hash 并返回有效的视图引用
  // 此步骤在主CUDA流上执行，确保GPU哈希表与最新TSDF层数据同步
  // getGpuLayerView内部会调用synchronize()，因此返回时哈希已是最新
  const GPULayerView<TsdfBlock>& gpu_layer_view = tsdf_layer.getGpuLayerView(cuda_stream);

  // GPU Output space
  device_vector<Vector3f> d_terrain_points_(total_points_);
  device_vector<bool> d_point_validity_(total_points_);
  // 使用独立非阻塞流初始化设备内存，避免与主映射流阻塞
  d_terrain_points_.setZeroAsync(height_scan_stream_);
  d_point_validity_.setZeroAsync(height_scan_stream_);

  // 配置CUDA内核参数
  const int threads_per_block = 256;
  const int num_blocks_needed = (total_points_ + threads_per_block - 1) / threads_per_block;

  // 启动合并CUDA内核：计算采样点 + DDA步进 + 子体素插值
  // 在独立非阻塞流上执行，不与主映射流(integrateDepth/updateEsdf)阻塞
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

  cudaError_t err = cudaPeekAtLastError();
  if (err != cudaSuccess) {
    RCLCPP_ERROR(rclcpp::get_logger("CudaVerticalRayCaster"),
                 "sampleTerrainKernel failed: %s", cudaGetErrorString(err));
    return false;
  }

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
  
  // fillInvalidElevationPoints(sample_points_z, h_point_validity_, x_steps, y_steps);

  return true;
}

bool CudaVerticalRayCaster::initializeGroundPlane(TsdfLayer& tsdf_layer,
                                                 const Transform& robot_pose,
                                                 const CudaStream& cuda_stream) {
  // 获取机器人位置
  Vector3f robot_position = robot_pose.translation();
  
  // 计算地平面高度（机器人下方0.12m处）
  const float ground_height = robot_position.z() - 0.12f;
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
              "Ground plane initialized at height %.3f m (after %d samples)",
              ground_height, sample_count_);
  
  return true;
}

}  // namespace conversions
}  // namespace nvblox
