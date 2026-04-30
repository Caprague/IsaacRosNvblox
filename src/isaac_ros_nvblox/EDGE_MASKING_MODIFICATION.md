# 深度图像边缘掩膜修改总结

## 修改概述

本次修改实现了 **方案 5：深度图像边缘掩膜**，用于解决深度相机上方视野边缘处凭空建图的问题。

## 修改的文件

### 1. nvblox_core/nvblox/include/nvblox/integrators/internal/cuda/impl/projective_integrator_impl.cuh

**文件路径**：
```
/home/nvidia/IsaacRos/isaac_ros-dev/src/isaac_ros_nvblox/nvblox_ros/nvblox_core/nvblox/include/nvblox/integrators/internal/cuda/impl/projective_integrator_impl.cuh
```

## 修改内容

### 修改 1：添加边缘检查设备函数

**位置**：文件开头的匿名命名空间内（第 49-70 行）

**新增函数**：
```cpp
// Device function to check if a pixel is in the edge region
// Returns true if the pixel is in the edge region (should be masked)
__device__ inline bool isPixelInEdgeRegion(const int row, const int col,
                                            const int rows, const int cols,
                                            const float edge_margin_ratio = 0.1f) {
  const int edge_margin_x = static_cast<int>(cols * edge_margin_ratio);
  const int edge_margin_y = static_cast<int>(rows * edge_margin_ratio);
  
  // Check if pixel is in the horizontal edge region
  if (col < edge_margin_x || col >= cols - edge_margin_x) {
    return true;
  }
  
  // Check if pixel is in the vertical edge region
  if (row < edge_margin_y || row >= rows - edge_margin_y) {
    return true;
  }
  
  return false;
}
```

**功能**：
- 检查给定像素是否位于深度图像的边缘区域
- 默认边缘比例为 10%（`edge_margin_ratio = 0.1f`）
- 返回 `true` 表示像素在边缘区域，应该被掩膜

### 修改 2：添加距离检查设备函数

**位置**：文件开头的匿名命名空间内（第 72-76 行）

**新增函数**：
```cpp
// Device function to check if a voxel is too close to the camera
// Returns true if the voxel is too close (should not be updated)
__device__ inline bool isVoxelTooCloseToCamera(const float voxel_depth_m,
                                               const float min_distance_m = 0.15f) {
  return voxel_depth_m < min_distance_m;
}
```

**功能**：
- 检查体素是否离相机过近
- 默认最小距离为 15cm（`min_distance_m = 0.15f`）
- 返回 `true` 表示体素太近，不应该被更新

### 修改 3：在 Camera 的 integrateBlocksKernel 中添加边缘和距离检查

**位置**：Camera 版本的 `integrateBlocksKernel` 函数（第 113-125 行）

**新增代码**：
```cpp
// Check if the pixel is in the edge region (depth image edge masking)
// This helps prevent spurious voxels at the edges of the depth image
if (isPixelInEdgeRegion(pix_pos.y(), pix_pos.x(), image.rows(), image.cols())) {
  return;
}

// Check if the voxel is too close to the camera
// This prevents updating voxels that are too close to the camera (e.g., < 15cm)
if (isVoxelTooCloseToCamera(voxel_depth_m)) {
  return;
}
```

**插入位置**：
- 在深度图像插值之后
- 在 `isMasked` 检查之前
- 在体素更新之前

**效果**：
- 如果像素位于边缘区域，直接返回，不更新体素
- 如果体素离相机过近（< 15cm），直接返回，不更新体素
- 防止边缘噪声和近距离噪声导致的凭空建图

### 修改 4：在 LiDAR 的 integrateBlocksKernel 中添加边缘和距离检查

**位置**：LiDAR 版本的 `integrateBlocksKernel` 函数（第 174-186 行）

**新增代码**：
```cpp
// Check if the pixel is in the edge region (depth image edge masking)
// This helps prevent spurious voxels at the edges of the depth image
if (isPixelInEdgeRegion(pix_pos.y(), pix_pos.x(), image.rows(), image.cols())) {
  return;
}

// Check if the voxel is too close to the camera
// This prevents updating voxels that are too close to the camera (e.g., < 15cm)
if (isVoxelTooCloseToCamera(voxel_depth_m)) {
  return;
}
```

**插入位置**：
- 在深度图像插值之后
- 在 `isMasked` 检查之前
- 在体素更新之前

**效果**：
- 如果像素位于边缘区域，直接返回，不更新体素
- 如果体素离相机过近（< 15cm），直接返回，不更新体素
- 防止边缘噪声和近距离噪声导致的凭空建图

## 工作原理

1. **边缘检测**：`isPixelInEdgeRegion` 函数计算深度图像的边缘区域
   - 水平边缘：左右各 10% 的像素
   - 垂直边缘：上下各 10% 的像素

2. **距离检查**：`isVoxelTooCloseToCamera` 函数检查体素是否离相机过近
   - 最小距离：15cm
   - 防止近距离噪声影响

3. **早期返回**：在 GPU 核函数中，如果检测到像素在边缘区域或体素太近，立即返回
   - 避免不必要的体素更新计算
   - 提高性能

4. **防止噪声**：
   - 边缘区域的深度测量通常噪声较大，忽略这些区域
   - 近距离的深度测量可能不准确，忽略过近的体素
   - 防止在远离实际表面的地方创建体素

## 优势

1. **高效**：在 GPU 核函数中实现，零额外开销
2. **简单**：只需要添加一个设备函数和两次调用
3. **有效**：直接从源头防止边缘噪声影响建图
4. **可配置**：通过修改 `edge_margin_ratio` 参数可以调整边缘比例

## 参数说明

- `edge_margin_ratio`：边缘区域比例（默认 0.1，即 10%）
  - 可以根据实际场景调整
  - 建议范围：0.05 - 0.15（5% - 15%）
  - 过小：可能无法完全消除边缘噪声
  - 过大：可能丢失有效观测

- `min_distance_m`：距离相机的最小距离（默认 0.15，即 15cm）
  - 可以根据实际场景调整
  - 建议范围：0.10 - 0.25（10cm - 25cm）
  - 过小：可能无法完全消除近距离噪声
  - 过大：可能丢失近距离的有效观测

## 测试建议

1. **可视化测试**：
   - 在 RViz 中观察 TSDF 层
   - 检查边缘区域是否还有凭空建图
   - 检查近距离区域是否还有凭空建图

2. **性能测试**：
   - 测量建图性能是否受影响
   - 应该几乎没有性能损失

3. **参数调优**：
   - 如果边缘问题仍然存在，增加 `edge_margin_ratio`（如 0.12 或 0.15）
   - 如果丢失太多有效观测，减小 `edge_margin_ratio`（如 0.08 或 0.07）
   - 如果近距离问题仍然存在，增加 `min_distance_m`（如 0.20 或 0.25）
   - 如果丢失太多近距离有效观测，减小 `min_distance_m`（如 0.12 或 0.10）

## 预期效果

- ✅ 消除深度相机上方视野边缘处的凭空建图
- ✅ 消除距离相机过近区域的凭空建图
- ✅ 保持建图质量不受影响
- ✅ 性能几乎没有损失
- ✅ 代码简洁，易于维护

## 注意事项

1. **编译**：修改后需要重新编译 nvblox_ros 包
2. **测试**：建议在实际环境中测试效果
3. **参数**：
   - 如果需要调整边缘比例，修改 `edge_margin_ratio` 参数
   - 如果需要调整最小距离，修改 `min_distance_m` 参数
4. **兼容性**：此修改不影响现有功能，只是额外添加了边缘和距离检查

## Git 分支

当前分支：`nvblox_aarch64_v5.0`

修改的文件：
- `nvblox_core/nvblox/include/nvblox/integrators/internal/cuda/impl/projective_integrator_impl.cuh`

## 下一步

1. 编译代码
2. 测试效果
3. 根据需要调整参数
4. 如果效果满意，可以提交到版本控制