# nvblox 库级接口

本页简要介绍如何在库级别与 nvblox 交互。如需查看 Doxygen 生成的 API 文档，请访问我们的 [our readthedocs page](https://nvblox.readthedocs.io/en/latest/index.html) 页面。

## 高级接口

最高级别的接口是 [Mapper](@ref nvblox::Mapper) 类。

```bash
const float voxel_size_m = 0.05;
const MemoryType memory_type = MemoryType::kDevice;
Mapper mapper(voxel_size_m, memory_type);
```

上述代码创建了一个 Mapper 实例，同时分配了一个空地图。这里我们指定体素大小为 5 厘米，并且存储在 GPU（设备）上。

Mapper 类提供了 [integrateDepth](@ref nvblox::Mapper::integrateDepth) 和 [integrateColor](@ref nvblox::Mapper::integrateColor) 方法，用于将深度和彩色图像添加到重建中：

```bash
mapper.integrateDepth(depth_image, T_L_C, camera);
mapper.integrateColor(color_image, T_L_C, camera);
```

用户需要向 nvblox 提供输入的 [image](@ref nvblox::DepthImage) `depth_image`、[camera pose](@ref nvblox::Transform) `T_L_C` 以及 [camera intrinsic model](@ref nvblox::Camera) `camera`。

上述函数调用将观测数据整合到 3D TSDF 体素网格中。TSDF 通常不是最终所需的输出，我们通常希望从中生成用于路径规划的欧几里得符号距离函数（ESDF），或生成网格以查看重建结果。Mapper 包含了实现这些功能的方法：[updateEsdf](@ref nvblox::Mapper::updateEsdf) 和 [updateMesh](@ref nvblox::Mapper::updateMesh)：

```bash
mapper.updateEsdf();
mapper.updateMesh();
```

这里的"update"表示这些函数并非从头开始生成网格或 ESDF，而是只更新必要的部分。

随后，我们可以使用 [outputMeshLayerToPly](@ref nvblox::io::outputMeshLayerToPly) 将网格保存为 `.ply` 文件：

```bash
io::outputMeshLayerToPly(mapper.mesh_layer(), "/path/to/my/cool/mesh.ply");
```

## 访问体素

如果您将 nvblox 作为库使用，可能希望直接操作体素。

体素存储在 [Layer](@ref nvblox::Layer) 类中。一个地图由多个 Layer 组成，这些 Layer 是存储不同类型体素的协同定位的体素网格。一个典型的地图可能包含 [TSDF](@ref nvblox::TsdfLayer)、[ESDF](@ref nvblox::EsdfLayer) 和 [Color](@ref nvblox::ColorLayer) 等 Layer。

Layer 提供了体素访问方法 [getVoxels](@ref nvblox::VoxelBlockLayer::getVoxels) 和 [getVoxelsGPU](@ref nvblox::VoxelBlockLayer::getVoxelsGPU)。这些方法会向调用者返回一个存储在 GPU 或 CPU 上的体素向量。

调用这些函数需要 GPU 运行一个内核（kernel）来从体素网格中检索体素并将其值复制到输出向量中。在 `getVoxels` 中，我们还需要将体素从 GPU 内存复制回主机（CPU）内存。

使用上述函数获取体素在内部是一个多步骤过程。该函数必须：
1.  调用一个内核，将查询位置转换为体素内存地址。
2.  将体素复制到输出向量中。
3.  我们还需要选择性地将输出向量从设备内存复制到主机内存。

因此，追求最高查询速度的高级用户应该直接在 GPU 内核中访问体素。下一节将讨论这个过程。

## 在 GPU 上访问体素

如果您希望编写直接使用体素值的高性能代码，您很可能需要在 GPU 内核中访问体素。

我们通过一个略微简化的 `getVoxels` 函数版本来说明如何实现（以下代码为示例，具体实现请参考库源码）：

```cpp
// 示例内核函数，用于查询体素
__global__ void queryVoxelsKernel(
    int num_queries, Index3DDeviceHashMapType<TsdfBlock> block_hash,
    float block_size, const Vector3f* query_locations_ptr,
    TsdfVoxel* voxels_ptr, bool* success_flags_ptr) {
  const int idx = threadIdx.x + blockIdx.x * blockDim.x;
  if (idx >= num_queries) {
    return;
  }
  const Vector3f query_location = query_locations_ptr[idx];

  TsdfVoxel* voxel;
  if (!getVoxelAtPosition<TsdfVoxel>(block_hash, query_location, block_size,
                                     &voxel)) {
    success_flags_ptr[idx] = false;
  } else {
    success_flags_ptr[idx] = true;
    voxels_ptr[idx] = *voxel;
  }
}

// 封装函数，用于在GPU上获取体素
void getVoxelsGPU(
    const TsdfLayer& layer,
    const device_vector<Vector3f>& positions_L,
    device_vector<TsdfVoxel>* voxels_ptr,
    device_vector<bool>* success_flags_ptr) {

  const int num_queries = positions_L.size();

  voxels_ptr->resize(num_queries);
  success_flags_ptr->resize(num_queries);

  constexpr int kNumThreads = 512;
  const int num_blocks = (num_queries + kNumThreads - 1) / kNumThreads;

  GPULayerView<TsdfBlock> gpu_layer_view = layer.getGpuLayerView(CudaStreamOwning());

  queryVoxelsKernel<<<num_blocks, kNumThreads>>>(
      num_queries, gpu_layer_view.getHash().impl_, layer.block_size(),
      positions_L.data(), voxels_ptr->data(), success_flags_ptr->data());
  checkCudaErrors(cudaDeviceSynchronize());
  checkCudaErrors(cudaPeekAtLastError());
}
```

在上述代码中，第一个关键步骤是获取代表地图的哈希表的 GPU 视图：

```cpp
GPULayerView<TsdfBlock> gpu_layer_view = layer.getGpuLayerView(CudaStreamOwning());
```

在内核中，使用这个哈希表将 3D 查询位置转换为体素的内存地址：
```cpp
TsdfVoxel* voxel;
if (!getVoxelAtPosition<TsdfVoxel>(block_hash, query_location, block_size, &voxel)) {
    // 处理获取失败的情况
} else {
    // 成功获取到体素指针，可以访问体素数据
    voxels_ptr[idx] = *voxel;
}
```
如果体素已被分配，getVoxelAtPosition 函数会将指向该体素的指针放入 `voxel` 变量并返回 true。

有关在 GPU 上查询体素的小型示例应用程序，请参阅 `/nvblox/examples/src/esdf_query.cu` 。
