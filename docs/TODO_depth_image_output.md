# TODO: Bridge直接输出Depth Image，消除nvblox冗余重投影

## 现状问题

mid360 bridge 与 nvblox 之间存在功能重合，导致每帧约68ms的冗余GPU计算。

### 数据流（当前）

```
Livox原始点云(unstructured)
  → bridge: 点→角度分格(原子聚合)→反算xyz→输出结构化PointCloud2(Cartesian xyz)
    → nvblox conversion: xyz→重新投影回角度格→depth image
      → nvblox integration: depth image积分(仅4.8ms)
```

bridge已将点云按仰角/方位角分好格，但输出的是Cartesian坐标点云。
nvblox拿到后又将xyz重新投影回角度格做depth image——**反向重投影，等于白做一遍**。

### 性能数据（900×16分辨率下）

| 阶段 | 耗时 |
|---|---|
| ros/lidar/conversion（冗余重投影） | **68.5ms**（均值），最大142ms |
| ros/lidar/integration（实际积分） | 4.8ms |
| 点云融合总延迟 | 189ms |

conversion占总延迟的96%，而integration本身仅需4.8ms。

## 目标方案

bridge直接输出depth image（2D浮点图像，每像素=距离值），nvblox跳过conversion直接进integration。

### 目标数据流

```
Livox原始点云(unstructured)
  → bridge: 点→角度分格(原子聚合)→直接输出depth image
    → nvblox integration: depth image积分(4.8ms)
```

预期收益：每帧节省~68ms，融合延迟从189ms降至~20ms量级。

## 需要修改的部分

### 1. mid360_to_nvblox_bridge

- **新增输出**：除现有PointCloud2外，增加 `sensor_msgs::msg::Image` (32FC1) 的depth image输出
- **修改GPU kernel**：`reconstructPointcloudKernel` 当前从grid反算xyz输出点云；
  需增加一条路径：直接将grid中的range值写入2D image buffer
- **新增话题**：如 `/lidar/depth_image`
- **新增camera_info**：nvblox的Lidar intrinsics需要通过CameraInfo或参数传递，
  需要发布对应的 `sensor_msgs::msg::CameraInfo`（含Lidar内参）

### 2. nvblox_ros

- **新增输入路径**：在 `NvbloxNode` 中增加depth image + camera_info的订阅
  （可复用现有depth图像处理管线，区别在于使用Lidar内参而非相机内参）
- **修改processLidarPointcloud**：当depth image可用时，跳过
  `checkLidarPointcloud` + `depthImageFromPointcloudGPU`，直接使用
  已有的depth image进入 `integrateLidarDepth`
- 或者：新增独立的 `processLidarDepthImage` 方法，避免修改现有逻辑

### 3. 参数同步

- bridge的 `virtual_lidar_width/height`、FOV、range 需与 nvblox 的
  `lidar_width/height`、FOV、range 保持一致
- 若通过CameraInfo传递，可减少手动同步负担

## 参考

- nvblox conversion实现：`nvblox_ros/src/lib/conversions/pointcloud_conversions.cu`
- bridge GPU实现：`mid360_to_nvblox_bridge/src/cuda/bridge_converter_gpu.cu`
- nvblox Lidar内参：`nvblox_ros/include/nvblox_ros/node_params.hpp` (lidar_width/height/FOV)
- bridge配置：`mid360_to_nvblox_bridge/config/mid360_bridge_gpu.yaml`
