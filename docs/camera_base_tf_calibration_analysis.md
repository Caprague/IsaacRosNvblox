# 相机→底盘TF自动标定分析

## 问题背景

当前 `odom_horizontal_transform` 包中的 `publish_camera_inverse_transform()` 硬编码了 `camera_link → base_link` 的旋转分量 `(w=1, x=0, y=-0.11, z=0)`，隐含假设相机水平朝前安装（约12.6°俯仰偏移）。但实际使用中相机俯仰角可变，导致：

- 相机倾斜安装后，硬编码的旋转不再正确
- 下游节点（nvblox、HeightScan）使用的TF链含错误的相机姿态
- 影响建图质量和地形采样精度

---

## 问题一：自动计算 camera_link → base_link 旋转

### 1.1 核心思路：双IMU重力对齐

系统中已有两个IMU：

| IMU | 位置 | 话题 | 测量量 |
|-----|------|------|--------|
| 相机IMU | Realsense D435i内部 | `/camera/imu` | 相机坐标系下的重力方向 `g_cam` |
| 底盘IMU | 机器人base中心附近 | 待配置 | 底盘坐标系下的重力方向 `g_base` |

**关键公式**：从两个IMU的重力测量恢复相对旋转

```
g_cam = R_world_cam^T · g_world    (相机IMU测量)
g_base = R_world_base^T · g_world  (底盘IMU测量)

则: R_base_cam = R_world_base^T · R_world_cam
```

即：分别计算两个IMU到水平坐标系的旋转，然后组合得到相机到底盘的相对旋转。

### 1.2 具体算法

**Step 1**: 从相机IMU加速度计算 `R_world_cam`（当前代码已有此逻辑）

```
measured_gravity_cam = avg(camera_accel_samples).normalized()
target_gravity = (0, -1, 0)
R_world_cam = AxisAngle(measured_gravity_cam → target_gravity)
```

**Step 2**: 从底盘IMU加速度计算 `R_world_base`（新增逻辑，与Step 1相同算法）

```
measured_gravity_base = avg(base_accel_samples).normalized()
target_gravity = (0, -1, 0)
R_world_base = AxisAngle(measured_gravity_base → target_gravity)
```

**Step 3**: 组合得到相机到底盘的相对旋转

```cpp
Eigen::Matrix3d R_world_cam = q_cam.toRotationMatrix();
Eigen::Matrix3d R_world_base = q_base.toRotationMatrix();
Eigen::Matrix3d R_base_cam = R_world_base.transpose() * R_world_cam;
Eigen::Quaterniond q_base_cam(R_base_cam);
```

**Step 4**: 取逆得到 camera_link → base_link 的旋转

```cpp
Eigen::Quaterniond q_cam_base = q_base_cam.inverse();
// 即 camera_link → base_link 的旋转四元数
```

### 1.3 平移分量处理

平移分量（相机光学中心到底盘中心的位移）**无法从IMU重力测量中恢复**，但：

- 已有良好的初值估计（误差3cm内）：`(0.34, 0, 0.09)` → 逆变换 `(-0.34, 0, -0.09)`
- 3cm平移误差对HeightScan等下游功能影响很小（采样点间距通常5-10cm）
- 短期内继续使用硬编码平移，后续可通过标定精化

### 1.4 Yaw不可观问题

单姿态下，重力测量只能约束 **roll 和 pitch**（2个自由度），**yaw不可观**。但对于本场景：

- 相机与底盘通过刚体连接，yaw关系固定（相机始终朝前）
- 当前硬编码值中 yaw 分量也接近0
- 可以将 yaw 作为参数保留，默认值0，用户可手动微调

### 1.5 代码修改方案

```
现有流程:
  订阅 /camera/imu → 收集3s → 计算R_world_cam → 发布TF

修改后流程:
  订阅 /camera/imu + /base_imu → 同时收集3s → 分别计算R_world_cam和R_world_base
  → R_base_cam = R_world_base^T · R_world_cam → 发布TF(旋转自动计算)
```

**需要修改的内容**：

| 修改项 | 说明 |
|--------|------|
| 新增参数 | `base_imu_topic`（底盘IMU话题名） |
| 新增订阅 | `base_imu_subscription_`，回调中收集底盘加速度样本 |
| 新增成员 | `base_accel_samples_`，`measured_gravity_base_` |
| 修改 `process_imu_data()` | 并行处理两组IMU数据，计算相对旋转 |
| 修改 `publish_camera_inverse_transform()` | 旋转分量使用计算值替代硬编码值 |
| 修改 launch 文件 | 添加 `base_imu_topic` 参数 |

---

## 问题二：在线自适应标定方法

### 2.1 方案对比

| 方案 | 精度 | 旋转 | 平移 | 额外硬件 | 复杂度 | 是否适合 |
|------|------|------|------|----------|--------|----------|
| **A. 双IMU重力对齐** | 高(roll/pitch) | ✓ | ✗ | 无 | 低 | ★★★★★ |
| **B. 手眼标定(AprilTag)** | 高 | ✓ | ✓ | AprilTag标定板 | 中 | ★★★ |
| **C. 轨迹对手眼标定** | 中 | ✓ | ✓ | 无(需运动) | 高 | ★★ |
| **D. 双IMU + 多姿态Wahba** | 高(含yaw) | ✓ | ✗ | 无(需换姿态) | 中 | ★★★ |
| **E. 深度点云ICP对齐** | 中 | ✓ | ✓ | 无 | 高 | ★★ |

### 2.2 推荐方案：双IMU重力对齐（方案A）+ 平移初值

**理由**：

1. **零额外硬件**：两个IMU均已存在，无需标定板或特殊装置
2. **初始化阶段完成**：与现有3秒数据收集流程完全兼容，无需额外步骤
3. **精度足够**：
   - Roll/Pitch精度 < 0.5°（静态加速度计噪声经3秒平均后极低）
   - 平移使用硬编码初值，3cm误差对heightscan可接受
4. **复杂度最低**：在现有代码基础上增加约50行
5. **一次标定终身使用**：相机启动后姿态不再变，无需持续更新

### 2.3 可选精化：AprilTag手眼标定（方案B）

如果需要更高精度（平移+旋转全6DOF），可使用AprilTag方案：

**流程**：
1. 在机器人前方放置AprilTag（位置已知或任意）
2. 启动 `isaac_ros_apriltag` 节点检测标签
3. 使用 `easy_handeye2` 或 OpenCV `calibrateHandEye()` 计算相机到底盘的外参
4. 将结果写入参数或URDF

**适用场景**：部署前一次性标定，不在运行时执行。这与"启动后姿态不变"的约束一致。

**ROS 2 可用工具**：
- `isaac_ros_apriltag`：AprilTag检测（NVIDIA GPU加速）
- `easy_handeye2`：手眼标定（Tsai-Lenz算法，Humble支持）
- OpenCV `calibrateHandEye()`：5种算法可选

### 2.4 多姿态Wahba问题（方案D）

如果需要从纯IMU数据恢复yaw（不依赖先验或视觉），可在初始化时让机器人变换2-3个不同姿态：

**原理**：
- 单姿态：重力仅约束2个旋转DOF（roll+pitch），yaw不可观
- 多姿态：每个姿态提供一组重力方向对，N≥2组非共线对可求解完整3DOF旋转
- 这是经典的Wahba问题，可用SVD或四元数法闭式求解

**局限性**：
- 需要机器人能旋转到不同朝向（初始化阶段通常不具备）
- 增加了部署复杂度
- 对本场景性价比不高（yaw通常为0或已知）

### 2.5 不推荐方案及原因

**方案C（轨迹对手眼标定）**：需要机器人在初始化阶段做特定运动，且需要SLAM收敛后才能提供可靠的位姿估计。对于"启动即标定"的场景不够稳定。

**方案E（深度点云ICP对齐）**：需要已知环境的三维模型或预建地图，且ICP收敛性受初始值影响大。平移初值3cm精度虽好，但旋转初值可能偏差较大导致ICP失败。

---

## 3. 综合推荐实施路径

```
Phase 1（立即可做）:
  实现双IMU重力对齐 → 自动计算 camera_link → base_link 旋转
  平移继续使用硬编码初值

Phase 2（按需精化）:
  部署前使用AprilTag手眼标定 → 获得6DOF精确外参
  将结果写入参数文件，启动时加载

Phase 3（远期可选）:
  实现EKF/因子图在线估计 → 持续微调外参
  适用于长期运行中因振动导致的参数漂移
```

**Phase 1 的核心代码逻辑**（伪代码）：

```cpp
// 同时收集两组IMU数据（3秒）
imu_callback_camera → camera_accel_samples_
imu_callback_base   → base_accel_samples_

// 处理
process_imu_data() {
    // 原有逻辑：计算 R_world_cam
    q_cam = computeAlignmentRotation(camera_accel_samples_);  // 已有

    // 新增：计算 R_world_base
    q_base = computeAlignmentRotation(base_accel_samples_);    // 复用同一函数

    // 计算相对旋转
    R_base_cam = q_base.toRotationMatrix().transpose() * q_cam.toRotationMatrix();
    q_camera_to_base = Eigen::Quaterniond(R_base_cam).inverse();

    // 保存用于TF发布
    camera_base_rotation_ = q_camera_to_base;
}
```

---

## 4. 底盘IMU数据源

### 4.1 实际数据源

底盘IMU数据来自 **Unitree Go2 机器人的DDS话题 `rt/lowstate`**，数据格式为 `unitree_go::msg::dds_::LowState_`，其中 `imu_state().accelerometer()[3]` 包含三轴加速度。

### 4.2 实现方案：DDS→ROS2桥接

代码采用 **ROS 2话题订阅** 方式获取底盘IMU数据（参数 `base_imu_topic`），而非直接链接Unitree SDK2。原因：

1. **避免Cyclone DDS冲突**：ROS 2 Humble默认使用Cyclone DDS，Unitree SDK2也使用Cyclone DDS。同进程内两个DDS实例可能导致符号冲突
2. **架构解耦**：桥接节点独立运行，职责清晰
3. **灵活性**：可用任何方式提供底盘IMU话题（Unitree SDK2桥接、rosbag回放、仿真等）

### 4.3 桥接节点实现要点

需创建一个独立的ROS 2节点，功能：
1. 使用 `ChannelSubscriber<LowState_>` 订阅DDS话题 `rt/lowstate`
2. 提取 `imu_state()` 中的加速度、角速度、四元数
3. 转换为 `sensor_msgs/Imu` 消息发布到ROS 2话题（如 `/base_imu`）

桥接节点伪代码：
```cpp
// 使用Unitree SDK2
ChannelFactory::Instance()->Init(0, "eth0");
auto sub = make_shared<ChannelSubscriber<LowState_>>("rt/lowstate");
sub->InitChannel(callback, 1);

void callback(const void* msg) {
    auto* low_state = (LowState_*)msg;
    auto imu_msg = sensor_msgs::msg::Imu();
    imu_msg.linear_acceleration.x = low_state->imu_state().accelerometer()[0];
    imu_msg.linear_acceleration.y = low_state->imu_state().accelerometer()[1];
    imu_msg.linear_acceleration.z = low_state->imu_state().accelerometer()[2];
    // ... 角速度、四元数同理
    publisher->publish(imu_msg);
}
```

### 4.4 当前代码已实现的接口

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `base_imu_topic` | `""` (空) | 底盘IMU话题名，空=禁用双IMU标定 |
| `camera_base_x` | `0.34` | 相机在底盘坐标系中的X位置 |
| `camera_base_y` | `0.0` | 相机在底盘坐标系中的Y位置 |
| `camera_base_z` | `0.09` | 相机在底盘坐标系中的Z位置 |

当 `base_imu_topic` 为空时，回退到硬编码旋转四元数 `(w=1, x=0, y=-0.11, z=0)`。
