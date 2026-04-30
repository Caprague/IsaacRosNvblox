# 地平面初始化功能实现

## 修改概述

本次修改为 `CudaVerticalRayCaster` 类添加了地平面初始化功能，在首次采样时自动在机器人下方创建地平面体素栅格，方便后续采样。

## 修改的文件

### 1. 新增文件

- `nvblox_ros/include/nvblox_ros/conversions/vertical_ray_caster.hpp`
  - CudaVerticalRayCaster 类的头文件
  - 添加了 `ground_plane_initialized_` 标志位
  - 添加了 `initializeGroundPlane()` 函数声明
  - 修改了 `sampleTerrainPoints()` 函数签名（参数从 `const TsdfLayer&` 改为 `TsdfLayer&`）

- `nvblox_ros/include/nvblox_ros/conversions/vertical_ray_caster.cuh`
  - CUDA 头文件（包含 CUDA 内核函数声明）

- `nvblox_ros/src/lib/conversions/vertical_ray_caster.cu`
  - CudaVerticalRayCaster 类的实现文件
  - 实现了 `initializeGroundPlane()` 函数
  - 修改了 `sampleTerrainPoints()` 函数实现，添加地平面初始化检查

### 2. 修改的文件

- `nvblox_ros/include/nvblox_ros/layer_publishing.hpp`
  - 修改了 `publishLocomotionHeightScan_impl()` 函数签名（参数从 `const TsdfLayer&` 改为 `TsdfLayer&`）
  - 修改了 `publishNavigationHeightScan_impl()` 函数签名（参数从 `const TsdfLayer&` 改为 `TsdfLayer&`）

- `nvblox_ros/src/lib/layer_publishing.cpp`
  - 修改了 `publishLocomotionHeightScan_impl()` 函数实现
  - 修改了 `publishNavigationHeightScan_impl()` 函数实现

## 功能说明

### 地平面初始化参数

- **地平面高度**：机器人当前位置下方 0.15m
- **地平面大小**：2m x 2m 的正方形
- **体素距离**：-0.5 * voxel_size（负值表示占据）
- **体素权重**：5.0（高置信度）

### 工作流程

1. **首次调用**：当 `sampleTerrainPoints()` 第一次被调用时，检查 `ground_plane_initialized_` 标志位
2. **初始化地平面**：如果标志位为 false，调用 `initializeGroundPlane()` 函数
3. **创建体素**：在机器人下方 0.15m 处，2m x 2m 范围内创建体素栅格
4. **设置参数**：将体素的 distance 设置为负值（表示占据），weight 设置为高值（表示高置信度）
5. **更新 GPU**：调用 `getGpuLayerView()` 更新 GPU 层视图
6. **设置标志**：将 `ground_plane_initialized_` 设置为 true
7. **正常采样**：后续调用直接进行采样，不再初始化地平面

### 代码实现

#### initializeGroundPlane() 函数

```cpp
bool CudaVerticalRayCaster::initializeGroundPlane(TsdfLayer& tsdf_layer,
                                                 const Transform& robot_pose,
                                                 const CudaStream& cuda_stream) {
  // 获取机器人位置
  Vector3f robot_position = robot_pose.translation();
  
  // 获取体素大小和块大小
  const float voxel_size = tsdf_layer.voxel_size();
  const float block_size = tsdf_layer.block_size();
  
  // 地平面参数
  const float ground_height = robot_position.z() - 0.15f;  // 机器人下方0.15m
  const float plane_size = 2.0f;  // 2m x 2m的正方形
  const float half_size = plane_size / 2.0f;
  
  // 地平面体素参数
  const float ground_voxel_distance = -0.5f * voxel_size;  // 负距离表示占据
  const float ground_voxel_weight = 5.0f;  // 高置信度
  
  // 计算需要创建的体素范围
  const float min_x = robot_position.x() - half_size;
  const float max_x = robot_position.x() + half_size;
  const float min_y = robot_position.y() - half_size;
  const float max_y = robot_position.y() + half_size;
  
  // 遍历地平面范围内的所有体素
  for (float x = min_x; x < max_x; x += voxel_size) {
    for (float y = min_y; y < max_y; y += voxel_size) {
      // 计算体素中心位置
      Vector3f voxel_position(x, y, ground_height);
      
      // 获取块索引和体素索引
      Index3D block_idx, voxel_idx;
      getBlockAndVoxelIndexFromPositionInLayer(block_size, voxel_position, 
                                               &block_idx, &voxel_idx);
      
      // 分配块（如果不存在）
      auto block_ptr = tsdf_layer.allocateBlockAtIndex(block_idx);
      if (!block_ptr) {
        RCLCPP_ERROR(rclcpp::get_logger("CudaVerticalRayCaster"),
                     "Failed to allocate block at index [%d, %d, %d]",
                     block_idx.x(), block_idx.y(), block_idx.z());
        continue;
      }
      
      // 获取体素指针
      TsdfVoxel* voxel_ptr = &block_ptr->voxels[voxel_idx.z()][voxel_idx.y()][voxel_idx.x()];
      
      // 设置体素参数
      voxel_ptr->distance = ground_voxel_distance;
      voxel_ptr->weight = ground_voxel_weight;
    }
  }
  
  // 更新GPU层视图
  tsdf_layer.getGpuLayerView(cuda_stream);
  
  // 设置标志位
  ground_plane_initialized_ = true;
  
  RCLCPP_INFO(rclcpp::get_logger("CudaVerticalRayCaster"),
              "Ground plane initialized at height %.3f m with %d x %d voxels",
              ground_height, 
              static_cast<int>((max_x - min_x) / voxel_size),
              static_cast<int>((max_y - min_y) / voxel_size));
  
  return true;
}
```

#### sampleTerrainPoints() 函数修改

```cpp
bool CudaVerticalRayCaster::sampleTerrainPoints(...) {
  // 检查是否需要初始化地平面
  if (!ground_plane_initialized_) {
    if (!initializeGroundPlane(tsdf_layer, robot_pose, cuda_stream)) {
      RCLCPP_ERROR(rclcpp::get_logger("CudaVerticalRayCaster"),
                   "Failed to initialize ground plane");
      return false;
    }
  }
  
  // 正常采样逻辑...
}
```

## 注意事项

1. **参数修改**：需要将 `const TsdfLayer&` 改为 `TsdfLayer&`，因为需要修改图层内容
2. **线程安全**：地平面初始化只执行一次，后续调用不会重复初始化
3. **GPU 同步**：初始化后调用 `getGpuLayerView()` 确保 GPU 层视图更新
4. **错误处理**：如果块分配失败，会记录错误日志并跳过该体素

## 测试建议

1. **首次测试**：观察日志输出，确认地平面初始化成功
2. **采样测试**：检查采样结果是否正确反映地平面高度
3. **重复测试**：确认后续调用不会重复初始化地平面
4. **性能测试**：测量初始化开销，确保不影响实时性能

## 预期效果

- ✅ 首次采样时自动创建地平面体素栅格
- ✅ 后续采样能够正确采样到地平面
- ✅ 避免采样失败或返回无效数据
- ✅ 提高采样成功率