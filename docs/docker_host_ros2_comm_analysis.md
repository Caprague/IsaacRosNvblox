# Docker-主机间 ROS2 通信问题分析与解决方案

## 1. 问题描述

Docker 容器内运行 Isaac ROS 全栈（visual_slam, nvblox, safety_guardian），主机上运行 rl_sar 订阅 `/safety_guardian/status` 和 `/nvblox_node/locomotion_height_scan`。主机端订阅者始终收不到数据，打印 `Safety status: UNSAFE (count: ...)`。

## 2. 问题链分析

共发现 **3 个问题**，逐层递进：

### 2.1 Bug 1：safety_guardian 节点崩溃（时间源不匹配）

**现象**：节点启动约 1 秒后 crash：
```
terminate called after throwing an instance of 'std::runtime_error'
  what():  can't subtract times with different time sources [1 != 2]
```

**根因**：`safety_guardian_node.cpp` 中 `last_elevation_time_` 和 `elevation_calibration_start_` 未在构造函数中用节点时钟初始化。

- 默认构造的 `rclcpp::Time()` 使用 `RCL_SYSTEM_TIME`（源 2）
- `this->get_clock()->now()` 返回 `RCL_ROS_TIME`（源 1）
- summary timer 在收到第一条 elevation 消息前触发，做 `(now - last_elevation_time_).seconds()` 时时间源不匹配 → 抛异常

**结果**：节点启动即崩溃，主机端完全收不到消息。

**修复**（`safety_guardian_node.cpp` 构造函数）：
```cpp
// 修复前：未初始化（默认 SYSTEM_TIME）
this->elevation_calibrated_ = false;

// 修复后：用节点时钟初始化（ROS_TIME）
this->elevation_calibrated_ = false;
this->last_elevation_time_ = this->get_clock()->now();
this->elevation_calibration_start_ = this->get_clock()->now();
```

### 2.2 Bug 2：VSLAM 超时未传播到安全状态

**现象**：VSLAM TF 查找超时，summary 显示 VSLAM UNSAFE，但 Overall Safety Status 仍为 SAFE，持续发布 `true`。

**根因**：`monitorVSLAMStatus()` 中，TF 查找失败后直接 `return`，**没有调用 `updateSafetyStatus(false)`**：

```cpp
if (!success) {
    // ... 设置 vslam_safe_ = false ...
    return;  // ← 直接返回，未传播到 safety_status_
}
// ... 只有成功路径才调用 updateSafetyStatus ...
this->updateSafetyStatus(this->vslam_safe_);
```

`safety_status_` 只在 `updateSafetyStatus()` 中更新，而该函数仅在 TF 查找成功的路径末尾被调用。这意味着 VSLAM 超时永远不会导致发布 `false`。

**修复**（在 early return 前添加传播调用）：
```cpp
if (!success) {
    // ... 设置 vslam_safe_ = false ...
    this->updateSafetyStatus(this->vslam_safe_);  // ← 新增
    return;
}
```

### 2.3 Bug 3：FastDDS SHM 权限不匹配（主机收不到数据）

**现象**：修复前两个 bug 后，容器内 `ros2 topic echo` 能看到 `true`，但主机端 `ros2 topic echo` 完全收不到数据。`ros2 topic list` 能看到 topic（发现正常），但 `echo` 无数据（投递失败）。

**根因**：FastDDS SHM（DataSharing）传输层权限不匹配：

```
/dev/shm/ 权限现状：
-rw-r--r-- root   root    fastrtps_port741x    ← 容器进程(root, uid=0)创建
-rw-r--r-- nvidia docker   fastrtps_b9fb55435d  ← 主机进程(nvidia, uid=1000)创建
```

- 容器以 `root` 运行，umask `0022` → SHM 段权限 `644`（owner 读写，其他人只读）
- 主机以 `nvidia` 运行 → 对 root 拥有的 SHM 段只有读权限
- FastDDS DataSharing 需要**写入 ACK/流控**数据到 SHM 段 → 写入被拒绝 → 数据投递静默失败
- DDS 发现层（SPDP/SEDP）走 UDP 多播 → 正常，`ros2 topic list` 可见
- 数据投递层尝试 SHM → 权限不足 → `ros2 topic echo` 收不到

**初始 workaround**：在容器和主机两侧都设置 `FASTRTPS_DEFAULT_PROFILES_FILE` 指向 UDP-only profile，禁用 SHM。通信恢复，但引入新问题。

### 2.4 性能退化：全面禁用 SHM 导致 visual_slam 帧率下降

**现象**：全面禁用 SHM 后，visual_slam 频繁打印：
```
[WARN] Delta between current and previous frame [50-83 ms] is above threshold [34 ms]
```

**根因**：
- 容器内 nvblox ↔ visual_slam 等高吞吐通信原本走 SHM（零拷贝，低延迟）
- 禁用 SHM 后全部走 UDP → 序列化/反序列化经网络栈，CPU 开销激增
- Jetson Orin 有限 CPU 被额外 UDP 处理占用 → visual_slam 帧处理延迟飙升
- **关键对比**：跨边界流量极小（height_scan 748B@25Hz + Bool 1B@10Hz），容器内流量巨大（depth image, pointcloud, mesh 等）

## 3. 最终解决方案：非对称 FastDDS 传输配置

### 核心原理

FastDDS 传输协商机制：两个 Participant 发现彼此时交换可用 locator（SHM + UDP 或仅 UDP）。如果主机只声明 UDP locator，容器不会尝试 SHM → 无权限问题。

```
容器内 (root 进程, 默认 SHM+UDP)          主机 (nvidia 用户, UDP-only)
┌─────────────────────────┐               ┌──────────────────────┐
│  visual_slam ←─SHM─→ nvblox  (高性能)  │               │      │
│       │                   │   UDP 协商    │  rl_sar 订阅者       │
│  safety_guardian ──UDP──→│──────────────→│  收到 status/height  │
└─────────────────────────┘               └──────────────────────┘
```

### 具体配置

**容器侧**：不设置任何 FastDDS profile（保持默认 SHM+UDP）
```bash
# 确认环境变量为空
echo $FASTRTPS_DEFAULT_PROFILES_FILE   # 应为空
echo $FASTDDS_DEFAULT_PROFILES_FILE    # 应为空
```

**主机侧**：设置 UDP-only profile
```bash
# 已有文件：/home/nvidia/IsaacRos/isaac_ros-dev/fastrtps_disable_shm.xml
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/nvidia/IsaacRos/isaac_ros-dev/fastrtps_disable_shm.xml
```

主机 `~/.bashrc` 中已有此配置（第 260 行），每次开 shell 自动生效。

### UDP-only Profile 内容

`/home/nvidia/IsaacRos/isaac_ros-dev/fastrtps_disable_shm.xml`：
```xml
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <transport_descriptors>
        <transport_descriptor>
            <transport_id>udp_transport</transport_id>
            <type>UDPv4</type>
        </transport_descriptor>
    </transport_descriptors>
    <participant profile_name="default_participant" is_default_profile="true">
        <rtps>
            <useBuiltinTransports>false</useBuiltinTransports>
            <userTransports>
                <transport_id>udp_transport</transport_id>
            </userTransports>
        </rtps>
    </participant>
</profiles>
```

### Isaac ROS 自带同等 Profile

容器内 `/usr/local/share/middleware_profiles/rtps_udp_profile.xml` 功能相同（来自 `isaac_ros_common/docker/middleware_profiles/`），但此文件不应在容器侧使用。

## 4. 修改文件汇总

| 文件 | 修改内容 | 状态 |
|------|---------|------|
| `src/safety_guardian/src/safety_guardian_node.cpp` | Bug1: 初始化 elevation 时间变量为节点时钟；Bug2: VSLAM 超时路径添加 `updateSafetyStatus()` | 已修复并编译 |
| `fastrtps_disable_shm.xml` | 主机端 UDP-only profile | 已创建 |
| `~/.bashrc`（主机） | `export FASTRTPS_DEFAULT_PROFILES_FILE=...` | 已配置（第 260 行） |
| `sh_list/start_nvblox.sh` | 确认无 UDP profile 设置 | 无需修改 |

## 5. 验证步骤

```bash
# 1. 容器内：重启 safety_guardian 并确认 SHM 正常
ros2 topic hz /nvblox_node/locomotion_height_scan  # 应接近 25Hz

# 2. 主机端：确认环境变量
echo $FASTRTPS_DEFAULT_PROFILES_FILE
# 应输出：/home/nvidia/IsaacRos/isaac_ros-dev/fastrtps_disable_shm.xml

# 3. 主机端：验证能收到数据
ros2 topic echo /safety_guardian/status --once      # 应收到 true
ros2 topic echo /nvblox_node/locomotion_height_scan --once  # 应收到数据

# 4. 观察 visual_slam 帧率恢复正常（不再频繁出现 >34ms 警告）
```

## 6. 备选方案（如非对称配置仍有性能问题）

1. **降低跨边界 topic 频率**：height_scan 从 25Hz 降到 10Hz（修改 nvblox yaml 中 `publish_locomotion_height_scan_rate_hz`）
2. **主机侧使用 CycloneDDS**：`export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`（CycloneDDS 的 SHM 实现权限处理不同）
3. **容器以 nvidia 用户运行**：`docker run --user 1000:1001 ...` → SHM 段统一 nvidia 权限，双方均可读写（但可能影响容器内需要 root 权限的操作）
