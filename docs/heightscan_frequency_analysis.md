# HeightScan 发布频率问题根因分析与解决方案

## 一、测试数据摘要

| 指标 | set50Hz CSV (限频50Hz) | tick CSV (每次tick都执行) |
|------|----------------------|--------------------------|
| 样本数 | 6515 | 9579 |
| 平均频率 | **35.7 Hz** (目标50Hz) | **139.7 Hz** |
| 频率范围 | 0~55.5 Hz | 0~851 Hz |
| 平均间隔 | **32.1 ms** (目标20ms) | 12.6 ms |
| 间隔分布 | 20-25ms最多，>60ms有470次 | 极端不稳定，1ms~162ms |

---

## 二、根因分析（按影响程度排序）

### 根因1：单tick串行架构 —— 核心瓶颈

`tick()` 函数在 `MutuallyExclusive` 回调组中以 10ms 周期被定时器触发，**所有任务串行执行**：

```
tick() {
  processDepthQueue()        // 深度图积分 - 耗时大户
  processColorQueue()        // 彩色图积分
  processPointcloudQueue()   // 激光雷达积分 - 耗时大户
  decayTsdf()                // TSDF衰减
  clearMapOutsideOfRadius()  // 地图清理
  publishLocomotionHeightScan()  // ★ 高程采样
  processEsdf()              // ESDF更新 - 耗时大户
  publishLayers()            // 层可视化
  publishDebugVisualizations()
}
```

**问题机制**：`shouldProcess()` 判断是否执行基于 `time_now - time_last > 1/desired_freq`。但如果 tick 本身执行耗时超过 tick_period(10ms)，则定时器回调会延迟触发，导致：

- `publishLocomotionHeightScan` 的 `shouldProcess` 判定时刻 `now` 被前面的耗时操作推迟
- 当 tick 总耗时 > 20ms 时，即使 `shouldProcess` 返回 true，实际间隔也 > 20ms
- 从数据看：set50Hz 的平均间隔 32.1ms >> 目标 20ms，说明 **tick 内其他任务占据了大量时间**

**数据印证**：tick CSV 中平均间隔 12.6ms ≈ tick_period 10ms + 执行开销，但频率极度不稳定（1ms~162ms），说明 tick 执行时间剧烈波动。

---

### 根因2：共享CUDA流的同步阻塞 —— 关键加剧因素

整个节点使用**单一 `cuda_stream_`**（类型为 `CudaStreamOwning`，默认 `cudaStreamDefault` 即阻塞流），所有GPU操作共享此流：

- `integrateDepth()` / `integrateLidarDepth()` → 使用 `cuda_stream_`
- `updateEsdf()` → 使用 `cuda_stream_`，且内部有**多处显式 `synchronize()`**
- `decayTsdf()` → 使用 `cuda_stream_`，内部有显式 `synchronize()`
- `sampleTerrainPoints()` → 使用同一个 `cuda_stream_`，且有 `cudaStreamSynchronize()`

**关键问题**：`rayCastingVoxelsOnGPU` 核函数中每个线程执行 **串行光线追踪循环**（最多 `max_distance/voxel_size + 1 ≈ 60` 次迭代 + 3次精细采样），在 `sampleTerrainPoints()` 末尾还有**显式 `cudaStreamSynchronize()`**。这意味着：

1. 当 `processPointcloudQueue()` 或 `processDepthQueue()` 向 `cuda_stream_` 提交了大量异步GPU工作后
2. 后续 `sampleTerrainPoints()` 提交的核函数必须等待前面的GPU工作完成
3. `cudaStreamSynchronize()` 会阻塞CPU直到流上所有操作完成
4. 阻塞时间 = 前面积压的GPU工作 + 当前光线追踪工作量

**阻塞流(Default Stream)的额外问题**：`cudaStreamDefault` 创建的流会与默认流同步，这意味着任何使用默认流的操作（包括CUDA运行时内部操作）都会造成额外的隐式同步。

---

### 根因3：`shouldProcess` 的时间判定机制缺陷

```cpp
if (shouldProcess(now, publish_locomotion_height_scan_last_time_, 50.0)) {
    publishLocomotionHeightScan();
    publish_locomotion_height_scan_last_time_ = now;  // ← now是执行前的时刻
}
```

`now` 在 `shouldProcess` 判断时获取，但 `publishLocomotionHeightScan()` 执行需要数毫秒。两次 `now` 之差只反映了两次 tick 调用的时间间隔，而非两次实际发布完成的时间间隔。更重要的是：

- 如果某次 tick 耗时很长（如 ESDF 更新），`now` 已经被推迟
- `shouldProcess` 基于被推迟的 `now` 判断，看似间隔够了，但实际上距上次发布可能更长
- 且 `last_time` 记录的是 tick 开始时的 `now`，不是发布完成时的时间，导致下一次判定时时间差被低估

---

### 根因4：rayCastingVoxelsOnGPU 核函数效率问题

```cuda
for (int i = 0; i < kNumPoints; ++i) {  // kNumPoints ≈ 60
    // getVoxelAtPosition → GPU哈希表查询
    if (valid voxel found) {
        for (3 fine steps) {  // 精细采样
            getVoxelAtPosition(...)  // 再次哈希查询
        }
        break;
    }
}
```

- 每个线程最多 60+3=63 次 `getVoxelAtPosition`（即哈希表查询）
- locomotion 采样点数 = 17 × 11 = 187 个点
- 总哈希查询次数可达 187 × 63 ≈ 11,781 次
- 加上每次查询的非合并全局内存访问模式（哈希表随机访问），GPU利用率不高

---

### 根因5：不必要的CPU端开销

在 `publishLocomotionHeightScan_impl` 中：
- `getCpuLoadPercent()` 读取 `/proc/stat`
- `getMemUsageMB()` 读取 `/proc/meminfo`
- `getGpuLoadPercent()` 读取 `/sys/devices/platform/gpu.0/load`
- `logHeightScanStats()` 执行文件I/O（每次 `flush()`）

每次调用都执行这些 `/proc` 和 `/sys` 文件读取 + 磁盘写入，在高频(50Hz)下每秒增加 50 次文件系统操作。

---

## 三、解决方案

### 方案A：独立CUDA流 + 异步提交（推荐，效果最大）

**核心思路**：为 `sampleTerrainPoints` 创建独立的非阻塞CUDA流，避免与主映射流竞争。

- 创建一个独立的 `CudaStreamOwning(cudaStreamNonBlocking)` 专门用于 height scan
- `sampleTerrainPoints` 在独立流上提交核函数和 D2H 传输
- 使用 `cudaStreamSynchronize` 仅同步 height scan 流
- 这样 ESDF/TSDF 的 GPU 工作与 height scan 可以**并行执行**
- 代价：需要在 `rayCastingVoxelsOnGPU` 读取 TSDF 层时确保数据一致性（可在提交前同步主流，或使用 `cudaStreamWaitEvent` 做流间同步）

**预期提升**：消除根因2的阻塞，可将平均频率从 ~36Hz 提升到 ~45Hz+。

---

### 方案B：将 height scan 从 tick 串行链中解耦（效果最大但改动较大）

**核心思路**：将 `publishLocomotionHeightScan` 移到独立的定时器/线程中执行。

- 创建第二个 `CallbackGroup`（非 `MutuallyExclusive`），绑定独立的 `wall_timer`
- height scan 使用独立定时器按 50Hz 周期触发
- 需要对 TSDF 层的读取加读锁保护，防止与写操作冲突
- 可结合方案A使用独立CUDA流

**预期提升**：彻底解决根因1，height scan 不再被其他任务阻塞。

---

### 方案C：优化 rayCastingVoxelsOnGPU 核函数

- **减少迭代次数**：当前 max_casting_depth=3.0m, voxel_size=0.05m → 60次迭代。可先粗后细：以2倍voxel_size步长粗扫，命中后再精细搜索，将迭代减少到 ~30+3
- **合并访问**：当前 `getVoxelAtPosition` 对哈希表的访问是随机的，可考虑按 block 索引排序采样点，提高缓存命中率
- **避免重复 getGpuLayerView**：当前每次 `sampleTerrainPoints` 都调用 `tsdf_layer.getGpuLayerView(cuda_stream)`，这会更新 GPU hash，若 TSDF 层未变化可缓存

---

### 方案D：消除 tick 中的非必要开销

1. **移除日志 I/O**：将 `logHeightScanStats`、`getCpuLoadPercent` 等从热路径中移出，改为定时（如1Hz）写入，或异步写入
2. **移除冗余 RCLCPP_INFO**：`processPointcloudQueue` 中的 `RCLCPP_INFO(get_logger(), "cccccccc")` 每次 tick 都打印，应删除
3. **减少 PointCloud2 可视化发布**：`locomotion_hs_pc_publisher_` 的点云消息构建是 CPU 密集的，在不需要 Rviz 时关闭

---

### 方案E：降低其他任务的频率以腾出 tick 时间

- `integrate_lidar_rate_hz: 40` → 可降至 30
- `update_esdf_rate_hz: 5` → 保持（已很低）
- `decay_tsdf_rate_hz: 5` → 保持
- `publish_layer_rate_hz: 10` → 可降至 5（可视化非关键）
- `publish_debug_vis_rate_hz: 2` → 保持

---

### 方案F：shouldProcess 时间判定修正

将 `publish_locomotion_height_scan_last_time_` 的赋值时机从 tick 入口改为实际发布完成后：

```cpp
if (shouldProcess(now, last_time, 50.0)) {
    publishLocomotionHeightScan();
    last_time = this->get_clock()->now();  // 用发布完成后的时间
}
```

此改动虽不能提高绝对频率，但能使频率统计更准确、波动更可预测。

---

## 四、方案优先级建议

| 优先级 | 方案 | 预期效果 | 改动量 |
|--------|------|----------|--------|
| **P0** | A: 独立CUDA流 | 频率 +8~10Hz，稳定性大幅提升 | 小 |
| **P0** | D: 消除非必要开销 | 频率 +2~3Hz | 极小 |
| **P1** | C: 优化核函数 | 单次执行 -1~2ms | 中 |
| **P1** | F: 修正时间判定 | 稳定性提升 | 极小 |
| **P2** | B: 独立定时器/线程 | 彻底解耦，可达标50Hz | 大 |
| **P2** | E: 降低其他任务频率 | 间接腾出时间 | 极小 |

**建议先实施 A + D + F**（改动小、见效快），若仍不达标再实施 B 或 C。

---

## 五、总结

核心问题不是单一的，而是**架构层面的耦合**：

1. **串行 tick** 导致 height scan 被前面耗时任务推迟（根因1）
2. **共享 CUDA 流 + 显式同步** 导致 GPU 工作互相阻塞（根因2）
3. **核函数效率** 和 **不必要的 I/O** 进一步恶化单次执行耗时（根因4、5）

三者叠加，使得设定 50Hz 时实际只能达到 ~36Hz 且不稳定。单独解决任一因素都只能部分改善，**最有效的策略是解耦 CUDA 流（方案A）+ 减少开销（方案D）**，如需彻底达标则需架构解耦（方案B）。
