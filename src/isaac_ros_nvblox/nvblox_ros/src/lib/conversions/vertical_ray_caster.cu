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
#include <thrust/device_vector.h>
#include "nvblox/gpu_hash/internal/cuda/gpu_hash_interface.cuh"
#include "nvblox/gpu_hash/internal/cuda/gpu_indexing.cuh"

#include <queue>
#include <rclcpp/rclcpp.hpp>

namespace nvblox {
namespace conversions {

/**
 * @brief CUDA核心函数：预计算采样点全局坐标
 * @param d_sample_points 采样点坐标数组
 * @param x_steps 在x轴上的采样步数
 * @param y_steps 在y轴上的采样步数
 * @param range_x 采样范围在x轴上的长度
 * @param range_y 采样范围在y轴上的长度
 * @param resolution 采样网格的分辨率
 * @param robot_pos_x 机器人当前位置的x坐标
 * @param robot_pos_y 机器人当前位置的y坐标
 * @param robot_pos_z 机器人当前位置的z坐标
 * @param yaw_rot_00 yaw旋转矩阵的元素(0,0)
 * @param yaw_rot_01 yaw旋转矩阵的元素(0,1)
 * @param yaw_rot_10 yaw旋转矩阵的元素(1,0)
 * @param yaw_rot_11 yaw旋转矩阵的元素(1,1)
 * @param x_offset x轴上的偏移量
 * @param y_offset y轴上的偏移量
 * @param z_offset z轴上的偏移量
 */
__global__ void computeSamplePointsKernel(
  Vector3f* d_sample_points,
  const int x_steps, 
  const int y_steps,
  const float range_x,
  const float range_y,
  const float resolution,
  const float robot_pos_x,
  const float robot_pos_y,
  const float robot_pos_z,
  const float yaw_rot_00, const float yaw_rot_01,  // yaw旋转矩阵元素
  const float yaw_rot_10, const float yaw_rot_11,
  const float x_offset,
  const float y_offset,
  const float z_offset)
{
  // 计算全局线程索引
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  
  if (idx >= x_steps * y_steps) return;
  
  // 计算对应的i,j坐标
  int i = idx / x_steps;
  int j = idx % x_steps;
  
  // 计算相对坐标
  float rel_x = -range_x / 2 + j * resolution;
  float rel_y = -range_y / 2 + i * resolution;
  
  // 应用偏移和旋转
  float local_x = rel_x + x_offset;
  float local_y = rel_y + y_offset;
  
  // 旋转并添加机器人位置
  float global_x = robot_pos_x + yaw_rot_00 * local_x + yaw_rot_01 * local_y;
  float global_y = robot_pos_y + yaw_rot_10 * local_x + yaw_rot_11 * local_y;
  float global_z = robot_pos_z + z_offset;
  
  // 存储结果
  d_sample_points[idx] = nvblox::Vector3f(global_x, global_y, global_z);
}

// Cuda 核函数，批量光追采样地形体素，同时输出是否成功采样到体素
// 修改说明：将getVoxelsInLayerKernel的功能内联到光追采样循环中，避免嵌套启动内核。
// @param gpu_layer 输入的体素层
// @param sample_points 输入的采样点数组
// @param point_validity 输出的标志数组，指示是否成功采样到体素
// @param max_distance 最大光追采样距离
// @param occupied_threshold 体素栅格占据阈值
// @param block_size 体素块的大小
// @param voxel_size 体素栅格大小
// @param num_points 输入的采样点数量
__global__ void rayCastingVoxelsOnGPU(
  Index3DDeviceHashMapType<TsdfBlock> block_hash,
  Vector3f* d_sample_points,
  Vector3f* d_terrain_points,
  bool* d_point_validity,
  const float confidence_weight_threshold,
  const float block_size,
  const float voxel_size,
  const int num_points,
  const float max_distance) 
{
  const int idx = threadIdx.x + blockIdx.x * blockDim.x;
  if (idx >= num_points) {
    return;
  }

  // 计算单点的光追采样起始点和终止点
  nvblox::Vector3f beg_position = d_sample_points[idx];
  nvblox::Vector3f end_position = d_sample_points[idx];
  end_position.z() = beg_position.z() - max_distance;

  // 在单采样点位置，按体素大小生成序列子采样点
  int kNumPoints = static_cast<int>(max_distance / voxel_size) + 1;

  // 关键修改1：移除原有的设备向量声明和嵌套内核启动
  // 改为直接在循环内进行体素查询操作

  // 检查是否有体素被成功采样到
  bool has_valid_voxel = false;
  nvblox::Vector3f valid_terrain_point;

  for (int i = 0; i < kNumPoints; ++i)
  {
    // 计算当前采样点的位置
    nvblox::Vector3f current_position = beg_position + (end_position - beg_position) * (i / static_cast<float>(kNumPoints - 1));

    // 关键修改2：内联第一个核函数的逻辑
    // 直接查询当前采样点位置的体素
    TsdfVoxel* voxel_ptr = nullptr;
    // 假设 getVoxelAtPosition 函数可在设备端调用
    const bool flag = getVoxelAtPosition(block_hash, current_position, block_size, &voxel_ptr);

    // 如果成功获取体素且其占据概率超过阈值，则记录该点
    if (flag && voxel_ptr != nullptr && voxel_ptr->distance < 0.0f && voxel_ptr->weight > confidence_weight_threshold) {
      has_valid_voxel = true;
      valid_terrain_point = current_position; // 记录当前有效的 terrain point

      // 二次调整：尝试反向移动采样点，获取更精确的 terrain point
      // 定义精细采样的步长（单位：体素）
      const float fine_steps[] = {0.5f, 0.25f, 0.125f};
      const int num_fine_steps = sizeof(fine_steps) / sizeof(fine_steps[0]);
      // 向上精细采样
      for (int step_idx = 0; step_idx < num_fine_steps; ++step_idx) {
        // 计算微调后的位置（向上移动）
        nvblox::Vector3f fine_position = nvblox::Vector3f(valid_terrain_point.x(), 
                                                          valid_terrain_point.y(), 
                                                          valid_terrain_point.z() + fine_steps[step_idx] * voxel_size);
        
        // 检查微调后的位置
        TsdfVoxel* fine_voxel_ptr = nullptr;
        const bool fine_flag = getVoxelAtPosition(block_hash, fine_position, block_size, &fine_voxel_ptr);
        
        // 如果微调后的位置也有效，更新最精确的点
        if (fine_flag && fine_voxel_ptr != nullptr && fine_voxel_ptr->distance < 0.0f && fine_voxel_ptr->weight > confidence_weight_threshold) {
          valid_terrain_point = fine_position;
        } else {
          // 如果微调后的位置无效，尝试更小的步长
          continue;
        }
      }

      break; // 找到一个有效体素即可退出当前点的循环
    }
  }

  // 关键修改3：根据循环结果设置输出
  d_point_validity[idx] = has_valid_voxel;
  if (has_valid_voxel) 
  {
    d_terrain_points[idx] = valid_terrain_point;
  }
  else 
  {
    d_terrain_points[idx] = nvblox::Vector3f(beg_position.x(), beg_position.y(), 0);
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
  device_vector<Vector3f> d_sample_points_(total_points_);
  device_vector<Vector3f> d_terrain_points_(total_points_);  
  device_vector<bool> d_point_validity_(total_points_);
  // 使用独立非阻塞流初始化设备内存，避免与主映射流阻塞
  d_sample_points_.setZeroAsync(height_scan_stream_);
  d_terrain_points_.setZeroAsync(height_scan_stream_);
  d_point_validity_.setZeroAsync(height_scan_stream_);
  
  // 配置CUDA内核参数
  const int threads_per_block = 256;
  const int num_blocks_needed = (total_points_ + threads_per_block - 1) / threads_per_block;
  
  // 启动CUDA内核 - 计算采样点
  // 在独立非阻塞流上执行，不与主映射流(integrateDepth/updateEsdf)阻塞
  computeSamplePointsKernel<<<num_blocks_needed, threads_per_block, 0, height_scan_stream_>>>(
    d_sample_points_.data(),
    x_steps,
    y_steps,
    range_x,
    range_y,
    grid_resolution,
    robot_position.x(),
    robot_position.y(),
    robot_position.z(),
    yaw_rot_00, yaw_rot_01,
    yaw_rot_10, yaw_rot_11,
    x_offset,
    y_offset,
    z_offset);
  
  cudaError_t err1 = cudaPeekAtLastError();
  if (err1 != cudaSuccess) {
    RCLCPP_ERROR(rclcpp::get_logger("CudaVerticalRayCaster"), 
                 "computeSamplePointsKernel failed: %s", cudaGetErrorString(err1));
    return false;
  }
  
  // 启动CUDA内核 - 光线投射采样
  // 在独立非阻塞流上执行，不与主映射流阻塞
  rayCastingVoxelsOnGPU<<<num_blocks_needed, threads_per_block, 0, height_scan_stream_>>>(
    gpu_layer_view.getHash().impl_,
    d_sample_points_.data(),
    d_terrain_points_.data(),
    d_point_validity_.data(),
    confidence_weight_threshold_,
    block_size_,
    voxel_size_,
    total_points_,
    max_distance_
  );
  
  cudaError_t err2 = cudaPeekAtLastError();
  if (err2 != cudaSuccess) {
    RCLCPP_ERROR(rclcpp::get_logger("CudaVerticalRayCaster"), 
                 "rayCastingVoxelsOnGPU failed: %s", cudaGetErrorString(err2));
    return false;
  }
  
  // GPU -> CPU 数据传输（在独立流上异步执行）
  h_sample_points_.copyFromAsync(d_sample_points_, height_scan_stream_);
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
