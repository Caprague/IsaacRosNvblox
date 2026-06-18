# Mid360 + nvblox 快速参考卡

## 核心概念
```
Mid360非结构化点云 → Bridge节点网格化 → nvblox结构化融合
```

## 关键参数对照表

| 参数 | Bridge配置 | nvblox配置 | 说明 |
|------|-----------|-----------|------|
| 水平分辨率 | `virtual_lidar_width: 1800` | `lidar_width: 1800` | **必须匹配** |
| 垂直分辨率 | `virtual_lidar_height: 32` | `lidar_height: 32` | **必须匹配** |
| 最小距离 | `min_range_m: 0.5` | `lidar_min_valid_range_m: 0.5` | **必须匹配** |
| 最大距离 | `max_range_m: 30.0` | `lidar_max_valid_range_m: 30.0` | **必须匹配** |
| 最低仰角 | `min_elevation_deg: -7.0` | `min_angle_below_zero_elevation_rad: 0.122` | **必须匹配** |
| 最高仰角 | `max_elevation_deg: 52.0` | `max_angle_above_zero_elevation_rad: 0.908` | **必须匹配** |

## 快速启动命令

```bash
# 1. Mid360驱动
ros2 launch livox_ros_driver2 msg_MID360_launch.py

# 2. Bridge节点
ros2 launch mid360_to_nvblox_bridge mid360_bridge.launch.py

# 3. nvblox
ros2 launch nvblox_examples_bringup nvblox.launch.py setup_for_lidar:=True

# 4. 诊断（可选）
ros2 run mid360_to_nvblox_bridge bridge_diagnostics.py
```

## 话题列表

| 话题 | 类型 | 说明 |
|------|------|------|
| `/livox/lidar` | PointCloud2 | Mid360原始点云(输入) |
| `/lidar/pointcloud_structured` | PointCloud2 | 结构化点云(输出) |
| `/nvblox_node/mesh` | Mesh | 重建网格 |
| `/nvblox_node/map` | OccupancyGrid | 占据栅格地图 |

## 常用检查命令

```bash
# 查看话题频率
ros2 topic hz /lidar/pointcloud_structured

# 查看消息结构
ros2 topic echo /lidar/pointcloud_structured --once | head -20

# 检查参数
ros2 param get /mid360_bridge virtual_lidar_width
ros2 param get /nvblox_node lidar_width

# 查看节点状态
ros2 node info /mid360_bridge
```

## 性能预设

### 高质量（慢）
```yaml
width: 3600, height: 64
hole_filling: true, iterations: 3
aggregation: "median"
```
**延迟**: ~25ms | **CPU**: ~40%

### 平衡（推荐）
```yaml
width: 1800, height: 32
hole_filling: true, iterations: 2
aggregation: "mean"
```
**延迟**: ~6ms | **CPU**: ~18%

### 高速（快）
```yaml
width: 900, height: 16
hole_filling: false
aggregation: "min"
```
**延迟**: ~2ms | **CPU**: ~8%

## 故障排查速查

| 症状 | 可能原因 | 解决方案 |
|------|---------|---------|
| nvblox报错"intrinsics inconsistent" | 参数不匹配 | 对比并修正配置 |
| 输出点云稀疏 | FOV/Range设置 | 扩大FOV和范围 |
| 高CPU占用 | 分辨率过高 | 降低width/height |
| 建图质量差 | 点密度不足 | 启用hole_filling |
| 节点崩溃 | 配置文件错误 | 检查YAML语法 |

## 角度单位转换

```bash
# 度 → 弧度
弧度 = 度 × π / 180
例: 7° = 7 × 3.14159 / 180 = 0.122 rad

# 弧度 → 度
度 = 弧度 × 180 / π
例: 0.908 rad = 0.908 × 180 / 3.14159 = 52°
```

## 质量指标

| 指标 | 优秀 | 良好 | 可接受 | 需优化 |
|------|------|------|--------|--------|
| Valid Ratio | >85% | >75% | >65% | <65% |
| Latency | <5ms | <10ms | <15ms | >15ms |
| CPU Usage | <20% | <35% | <50% | >50% |
| Frame Rate | 30Hz | 20Hz | 10Hz | <10Hz |

## 一键诊断

```bash
# 下载并运行
wget https://raw.../diagnose.sh
chmod +x diagnose.sh
./diagnose.sh
```

## 重要提示

⚠️ **参数匹配是关键！** Bridge输出和nvblox输入的lidar参数必须完全一致，否则nvblox会拒绝点云。

✅ **验证成功标志**: 
- `bridge_diagnostics.py` 显示 valid_ratio > 70%
- nvblox无"inconsistent"错误日志
- RViz2中可看到规则网格状点云

📝 **配置文件位置**:
- Bridge: `~/ros2_ws/install/mid360_to_nvblox_bridge/share/.../config/`
- nvblox: `~/ros2_ws/install/nvblox_examples_bringup/share/.../config/`

---
**版本**: v1.0 | **更新**: 2024
