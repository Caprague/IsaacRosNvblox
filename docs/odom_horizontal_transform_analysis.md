# odom_horizontal_transform 包分析

## 1. 概述

**包名**: `odom_horizontal_transform`
**版本**: 0.0.0
**构建类型**: ament_cmake
**用途**: IMU水平对齐节点，基于加速度计数据计算重力方向，通过Axis-Angle旋转构建对齐四元数，并持续发布 `odom_horizontal → map` 等TF变换，使下游节点（如nvblox）能在水平坐标系中工作。

**核心思想**: 机器人在倾斜地面上启动时，IMU测量的重力方向不垂直向下。本节点计算一个旋转，将测量重力方向对齐到理想重力方向(0, -1, 0)，从而在TF树中插入一个"水平校正"层。

---

## 2. 节点生命周期

```
启动 → 延时等待(start_delay) → IMU数据收集(3s) → 数据处理与对齐计算 → 定时发布TF
```

| 阶段 | 行为 | 耗时 |
|------|------|------|
| 延时等待 | `std::this_thread::sleep_for(start_delay)`，等待IMU数据稳定 | 默认2.5s |
| 数据收集 | 订阅IMU话题，收集加速度计样本，定时器每100ms检查是否收集满3秒 | 3.0s |
| 数据处理 | 异常值过滤 → 平均 → 归一化 → Axis-Angle旋转 → RPY交换校正 → 验证 | 一次性 |
| TF发布 | 定时器回调以100Hz持续广播3条TF变换 | 持续运行 |

---

## 3. 算法详解

### 3.1 IMU数据收集

- 订阅 `/camera/imu` 话题（BestEffort QoS, KeepLast 10）
- 收集 `linear_acceleration` 三轴数据到 `std::vector<Eigen::Vector3d>`
- 收集时长: 3.0秒（硬编码 `collection_duration_`）
- 最少有效样本数: 10（硬编码 `min_valid_samples_`）

### 3.2 异常值过滤

```cpp
norm >= 9.0 && norm <= 10.0
```

只保留加速度模值在 [9.0, 10.0] m/s² 范围内的样本，剔除:
- 受动态加速度干扰的样本（运动、振动导致norm偏离9.8）
- 数据初始化阶段的异常读数

### 3.3 重力方向计算

1. 对过滤后样本取算术平均
2. 归一化得到测量重力方向向量 `measured_gravity`

### 3.4 Axis-Angle旋转计算

从 `measured_gravity` 旋转到 `target_gravity = (0, -1, 0)`:

```
rotation_axis = measured_gravity × target_gravity    (叉积)
rotation_angle = acos(measured_gravity · target_gravity)  (点积)
q_temp = AngleAxisd(rotation_angle, rotation_axis)
```

**特殊情况处理**:
- 叉积范数 < 1e-6 且点积 > 0: 已对齐，设为单位四元数
- 叉积范数 < 1e-6 且点积 < 0: 倒置，绕Z轴旋转180°

此旋转同时校正 **横滚(roll) 和 俯仰(pitch)**，因为Axis-Angle旋转是三维空间中最短路径旋转，不限定绕单个轴。

### 3.5 RPY交换校正

由于相机坐标系（Realsense D435i）与常规导航坐标系的轴定义不同，直接使用Axis-Angle四元数的欧拉角分解会产生错误的roll/pitch映射。代码执行了以下校正:

```cpp
// q_temp分解为RPY
corrected_roll_deg  = yaw_deg;                                          // 原yaw→新roll
corrected_pitch_deg = (roll_imu < -90°) ? roll_deg : (360° - roll_deg); // 条件选择
corrected_yaw_deg   = pitch_deg;                                        // 原pitch→新yaw

// 从校正后RPY重建四元数（ZYX顺序）
q = yawAngle * pitchAngle * rollAngle;
```

**坐标系定义** (Realsense D435i):
- X轴: 向右
- Y轴: 向下
- Z轴: 向前（从相机视角看）

### 3.6 验证

计算完成后进行两项验证:
1. **重力方向验证**: `q * measured_gravity` 应接近 `(0, -1, 0)`
2. **Z轴方向验证**: `q * (0, 0, 1)` 应在水平面内（Y分量接近0）

验证结果通过日志输出，供调试参考。

---

## 4. 发布的TF变换

### 4.1 odom_horizontal → map（水平校正变换）

| 字段 | 值 |
|------|------|
| parent_frame | `odom_horizontal` |
| child_frame | `map` |
| translation | (0, 0, 0) |
| rotation | alignment_quaternion_ |

**作用**: 将 `map` 坐标系中的倾斜姿态校正到 `odom_horizontal` 的水平姿态。nvblox建图时使用 `global_frame="odom_horizontal"`，通过TF2链 `odom_horizontal → map → ... → camera_link` 自动解析出水平校正后的位姿。

### 4.2 camera_link → base_link（逆变换）

| 字段 | 值 |
|------|------|
| parent_frame | `camera_link` |
| child_frame | `base_link` |
| translation | (-0.34, 0, -0.09) |
| rotation | (w=1, x=0, y=-0.11, z=0) |

**作用**: 定义相机到底盘的安装关系。注意这里的旋转分量y=-0.11表示绕Y轴有一个约12.6°的俯仰偏移，这可能是为了补偿相机安装角度。

> 注: `publish_camera_transform()` 方法（base_link→camera_link, translation=(0.34, 0, 0.09), 无旋转）已被注释掉，仅保留逆变换版本。

### 4.3 base_link → utlidar_lidar（激光雷达变换）

| 字段 | 值 |
|------|------|
| parent_frame | `base_link` |
| child_frame | `utlidar_lidar` |
| translation | (0.28945, 0, -0.04682) |
| rotation | (w=0.13132, x=0, y=0.99134, z=0) |

**作用**: 定义底盘到激光雷达的安装关系。对应欧拉角 (roll=-180°, pitch=15.091°, yaw=-180°)，表示雷达安装时有15°的俯仰倾斜和180°的翻转。

---

## 5. 与nvblox的集成

### 5.1 TF链路

```
odom_horizontal ──(alignment_quaternion_)──→ map ──(nvblox odometry)──→ odom ──→ camera_link
```

nvblox节点配置 `global_frame="odom_horizontal"` 时，TF2自动沿链路查找:
1. `odom_horizontal → map`: 本节点发布的水平校正旋转
2. `map → odom → camera_link`: nvblox自身的里程计链路

### 5.2 建图效果

- nvblox在 `odom_horizontal` 坐标系下构建的TSDF地图天然是水平的
- HeightScan采样同样在水平坐标系下进行，地面高程值不含倾斜分量
- 无需对nvblox代码做任何修改，完全依赖TF2的坐标变换机制

---

## 6. 参数配置

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `start_delay` | double | 0.0 | 延迟启动时间（秒），launch中设置为2.5 |
| `imu_topic` | string | `/camera/imu` | IMU数据话题 |
| `transform_publish_rate` | double | 100.0 | TF发布频率（Hz） |

**硬编码常量**:

| 常量 | 值 | 说明 |
|------|------|------|
| `collection_duration_` | 3.0s | IMU数据收集时长 |
| `min_valid_samples_` | 10 | 最少有效样本数 |
| 重力过滤范围 | [9.0, 10.0] m/s² | 加速度模值有效范围 |
| target_gravity | (0, -1, 0) | 目标重力方向（Y轴向下） |

---

## 7. Launch文件

`launch/test_imu_horizontal_align.launch.py` 同时启动:
1. **RealSense相机节点**: 启用红外/彩色/深度/IMU，深度模块90fps，IMU 200fps，unite_imu_method=2（合并加速度计+陀螺仪）
2. **IMU水平对齐节点**: start_delay=2.5s

---

## 8. 依赖关系

| 依赖 | 用途 |
|------|------|
| `rclcpp` | ROS 2 节点框架 |
| `sensor_msgs` | IMU消息类型 |
| `geometry_msgs` | TransformStamped消息 |
| `tf2_ros` | TF广播器 |
| `tf2_eigen` | TF与Eigen类型转换 |
| `Eigen3` | 线性代数运算（向量、四元数、旋转矩阵） |
