# 方案C3: tick() 多线程拆解实现总结

## 分支信息

- 基础分支: `aarch64_voxelmap_heightscan_5.6_pre` (commit `cadbfcb`)
- 开发分支: `feat/tick-thread-decomposition`

## 背景

方案A(独立CUDA流) + 方案B(height scan独立线程)实施后，height scan平均频率从35.7Hz提升至54.5Hz，但仍存在偶发频率不稳定。根因分析确认首要原因是：tick()中depth+color+pointcloud在一个`unique_lock`作用域内串行处理，持锁20-40ms，远超height scan的20ms周期，导致height scan线程的`shared_lock`阻塞等待。

## 修改文件

1. `nvblox_ros/include/nvblox_ros/nvblox_node.hpp`
2. `nvblox_ros/src/lib/nvblox_node.cpp`

## 线程架构

| 线程 | 入口函数 | 驱动方式 | 锁类型 | 职责 |
|------|---------|---------|--------|------|
| Integration | `integrationThreadFunc` | 事件驱动(cv + 10ms轮询) | `unique_lock`(每个queue独立作用域) | depth/color/pointcloud队列处理 + 服务请求处理 |
| Maintenance | `maintenanceThreadFunc` | 定时5Hz | `unique_lock`(每个操作独立作用域) | decayTsdf, decayDynamicOccupancy, clearMapOutsideRadius |
| Output | `outputThreadFunc` | 定时5Hz | `shared_lock` | processEsdf, publishLayers, publishDebugVisualizations |
| Height Scan | `heightScanThreadFunc` | 定时50Hz | `shared_lock` | height scan采样(已有，未修改) |
| tick() | ROS timer | 定时 | 无锁 | 仅打印统计信息(轻量) |

## 详细修改内容

### 1. nvblox_node.hpp

**新增成员声明:**

```cpp
// Dedicated thread for sensor integration (depth/color/pointcloud)
std::thread integration_thread_;
std::atomic<bool> integration_running_{false};
std::condition_variable integration_cv_;
std::mutex integration_cv_mutex_;

// Dedicated thread for maintenance tasks (decay, map clearing)
std::thread maintenance_thread_;
std::atomic<bool> maintenance_running_{false};

// Dedicated thread for output tasks (ESDF, layer publishing, debug vis)
std::thread output_thread_;
std::atomic<bool> output_running_{false};
```

**新增方法声明:**

```cpp
void integrationThreadFunc();
void maintenanceThreadFunc();
void outputThreadFunc();
```

### 2. nvblox_node.cpp

#### 构造函数 — 启动4个线程

在 `initializeMultiMapper()` 之后，`subscribeToTopics()` 之前启动所有线程:

```cpp
height_scan_running_ = true;
height_scan_thread_ = std::thread(&NvbloxNode::heightScanThreadFunc, this);

integration_running_ = true;
integration_thread_ = std::thread(&NvbloxNode::integrationThreadFunc, this);

maintenance_running_ = true;
maintenance_thread_ = std::thread(&NvbloxNode::maintenanceThreadFunc, this);

output_running_ = true;
output_thread_ = std::thread(&NvbloxNode::outputThreadFunc, this);
```

#### 析构函数 — 停止4个线程

在所有GPU/CUDA资源释放之前停止线程:

```cpp
integration_running_ = false;
integration_cv_.notify_all();
if (integration_thread_.joinable()) { integration_thread_.join(); }

maintenance_running_ = false;
if (maintenance_thread_.joinable()) { maintenance_thread_.join(); }

output_running_ = false;
if (output_thread_.joinable()) { output_thread_.join(); }

height_scan_running_ = false;
if (height_scan_thread_.joinable()) { height_scan_thread_.join(); }
```

#### integrationThreadFunc — 事件驱动的积分线程

```cpp
void NvbloxNode::integrationThreadFunc() {
  while (integration_running_) {
    // 等待新数据或停机信号 (10ms超时轮询)
    {
      std::unique_lock<std::mutex> cv_lock(integration_cv_mutex_);
      integration_cv_.wait_for(cv_lock, std::chrono::milliseconds(10),
        [this]() { return !integration_running_; });
    }
    if (!integration_running_) break;

    // 每个queue独立unique_lock作用域，释放锁间height scan可插入
    { unique_lock → processServiceRequestTaskQueue() }
    { unique_lock → processDepthQueue() }
    { unique_lock → processColorQueue() }
    { unique_lock → processPointcloudQueue() }
  }
}
```

**设计要点:**
- 每个queue独立加锁，不再合并为一个`unique_lock`作用域
- height scan线程在每个queue处理间隙可获取`shared_lock`
- 使用`condition_variable`在ROS回调pushOntoQueue后立即唤醒，10ms超时保底
- 最大持锁时间从20-40ms降至5-15ms(单个queue处理时间)

#### maintenanceThreadFunc — 定时5Hz维护线程

```cpp
void NvbloxNode::maintenanceThreadFunc() {
  const auto period = std::chrono::duration<double>(1.0 / 5.0);
  auto next_wake_time = steady_clock::now() + period;

  while (maintenance_running_) {
    sleep_until(next_wake_time);
    next_wake_time += period;
    if (!maintenance_running_) break;

    if (shouldProcess(...)) { unique_lock → decayTsdf() }
    if (shouldProcess(...)) { unique_lock → decayDynamicOccupancy() }
    if (shouldProcess(...)) { unique_lock → clearMapOutsideOfRadiusOfLastKnownPose() }
  }
}
```

**设计要点:**
- 每个操作独立`unique_lock`作用域，不合并
- `shouldProcess`按原参数控制实际执行频率
- decayTsdf/decayDynamicOccupancy/clearMapOutsideRadius均修改TSDF层，必须用`unique_lock`

#### outputThreadFunc — 定时5Hz输出线程

```cpp
void NvbloxNode::outputThreadFunc() {
  const auto period = std::chrono::duration<double>(1.0 / 5.0);
  auto next_wake_time = steady_clock::now() + period;

  while (output_running_) {
    sleep_until(next_wake_time);
    next_wake_time += period;
    if (!output_running_) break;

    if (shouldProcess(...)) { shared_lock → processEsdf() }
    if (shouldProcess(...)) { shared_lock → publishLayers() }
    if (shouldProcess(...)) { shared_lock → publishDebugVisualizations() }
  }
}
```

**设计要点:**
- 全部使用`shared_lock`，与height scan线程可并发执行
- processEsdf读TSDF写ESDF，publishLayers/publishDebugVis读层，均为只读或写独立层

#### tick() — 简化为仅打印统计

```cpp
void NvbloxNode::tick() {
  idle_timer_.Stop();
  timing::Timer tick_timer("ros/tick");
  timing::Rates::tick("ros/tick");

  // 所有繁重处理已移至独立线程:
  // - integrationThreadFunc(): depth/color/pointcloud
  // - maintenanceThreadFunc(): decay, map clearing
  // - outputThreadFunc(): ESDF, layer publishing, debug vis
  // - heightScanThreadFunc(): height scan at 50Hz

  // tick仅打印统计信息(轻量)
  auto & clk = *get_clock();
  if (params_.print_timings_to_console) { RCLCPP_INFO_STREAM_THROTTLE(...); }
  if (params_.print_rates_to_console)  { RCLCPP_INFO_STREAM_THROTTLE(...); }
  if (params_.print_delays_to_console) { RCLCPP_INFO_STREAM_THROTTLE(...); }

  idle_timer_.Start();
}
```

#### ROS回调 — 通知integration线程

在5个传感器回调的`pushOntoQueue`后添加:
```cpp
integration_cv_.notify_all();
```

涉及回调:
- `depthPlusMaskImageCallback`
- `depthImageCallback`
- `colorPlusMaskImageCallback`
- `colorImageCallback`
- `pointcloudCallback`

在4个服务回调的`pushOntoQueue`后同样添加:
- `savePly`
- `saveMap`
- `loadMap`
- `getEsdfAndGradientService`

## 锁保护策略

| 操作 | 线程 | 锁类型 | 作用域 | 原因 |
|------|------|--------|--------|------|
| processServiceRequestTaskQueue | Integration | unique_lock | 独立 | 可能修改TSDF |
| processDepthQueue | Integration | unique_lock | 独立 | 写TSDF |
| processColorQueue | Integration | unique_lock | 独立 | 写Color+GPU Hash |
| processPointcloudQueue | Integration | unique_lock | 独立 | 写TSDF |
| decayTsdf | Maintenance | unique_lock | 独立 | 修改TSDF+ESDF+Mesh |
| decayDynamicOccupancy | Maintenance | unique_lock | 独立 | 写Dynamic Occupancy+ESDF |
| clearMapOutsideRadius | Maintenance | unique_lock | 独立 | 释放TSDF块 |
| processEsdf | Output | shared_lock | 独立 | 读TSDF,写ESDF |
| publishLayers | Output | shared_lock | 独立 | 读层 |
| publishDebugVisualizations | Output | shared_lock | 独立 | 读参数 |
| publishLocomotionHeightScan | Height Scan | shared_lock | 独立 | 读TSDF |
| initializeGroundPlane | Height Scan | unique_lock | 独立(一次性) | 写TSDF |

## 关键优化效果

### 根因1解决: 缩短unique_lock阻塞窗口

**修改前**: tick()中 depth+color+pointcloud 合并一个`unique_lock`，持锁20-40ms

**修改后**: 每个queue独立`unique_lock`，单次持锁5-15ms，queue间释放锁

**效果**: height scan线程最大等待时间从20-40ms降至5-15ms，在20ms周期内可插入执行

### 线程间并发关系

```
Integration (unique_lock)  ←互斥→  Maintenance (unique_lock)
       ↓ 可并发                       ↓ 可并发
Output (shared_lock)       ←可并发→  Height Scan (shared_lock)
```

- Integration与Maintenance: 互斥(unique vs unique)
- Integration与Output: 互斥(unique vs shared)
- Integration与Height Scan: 互斥(unique vs shared)，但间隙释放锁
- Maintenance与Output: 互斥(unique vs shared)
- Maintenance与Height Scan: 互斥(unique vs shared)，但维护操作频率低(~5Hz)
- Output与Height Scan: 可并发(shared vs shared)

## 静态审查结果

| 检查项 | 结果 |
|--------|------|
| 声明一致性(4个线程函数hpp↔cpp) | PASS |
| 线程安全(无嵌套unique_lock) | PASS |
| 锁正确性(读写锁类型匹配) | PASS |
| 生命周期(构造后启动/析构前停止) | PASS |
| CV通知完整性(9个pushOntoQueue + 析构函数) | PASS |
| 空队列处理(processQueue直接return) | PASS |
| 无过期引用(tick注释与实际一致) | PASS |

## 时序依赖与CUDA流分析

### 数据依赖

- processDepthQueue与processPointcloudQueue写同一个`tsdf_layer` → 必须串行
- processColorQueue写独立`color_layer`但Mapper内部共享GPU Hash → 需unique_lock
- processEsdf读TSDF写ESDF → shared_lock即可
- static_mapper_与dynamic_mapper_完全独立 → 不同mapper操作可并发

### CUDA流分配

- 所有Mapper GPU操作(integrateDepth/Color/Lidar, decayTsdf, updateEsdf)内部调用`cuda_stream_->synchronize()`，为同步操作
- Height scan的ray casting使用独立非阻塞流`height_scan_stream_`
- foreground_mapper_与background_mapper_各有独立CUDA流，可GPU并发
- CPU端mutex释放时GPU工作已完成，不存在CPU/GPU竞态

### integrationThreadFunc的10ms轮询设计

- `wait_for(10ms)` + `notify_all()`组合：有数据时立即唤醒，无数据时最多10ms延迟
- 不在cv谓词中检查队列状态（避免在谓词中获取queue mutex增加复杂度和死锁风险）
- 空队列时`processQueue`直接return，开销极低
