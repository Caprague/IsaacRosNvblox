# Mid360 + nvblox 快速部署指南

## 一、系统要求

### 硬件
- **推荐**: Jetson Orin (8核) 或更高
- **最低**: Jetson Xavier NX (6核)
- **内存**: ≥8GB RAM
- **存储**: ≥10GB 可用空间

### 软件
- Ubuntu 20.04/22.04
- ROS2 Humble/Foxy
- CUDA 11.4+
- Livox SDK2 + ROS2 driver

## 二、安装步骤

### 步骤1: 安装依赖

```bash
# 更新系统
sudo apt update && sudo apt upgrade -y

# 安装ROS2依赖
sudo apt install -y \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-rclcpp \
    ros-${ROS_DISTRO}-rclcpp-components

# 安装Livox驱动 (如未安装)
cd ~/ros2_ws/src
git clone https://github.com/Livox-SDK/livox_ros_driver2.git
cd livox_ros_driver2
./build.sh humble  # 或你的ROS2版本
```

### 步骤2: 编译bridge包

```bash
cd ~/ros2_ws/src
# bridge包已在当前目录

cd ~/ros2_ws
colcon build --packages-select mid360_to_nvblox_bridge \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

source install/setup.bash
```

### 步骤3: 验证安装

```bash
# 检查节点是否可用
ros2 pkg list | grep mid360_to_nvblox_bridge

# 检查可执行文件
ros2 run mid360_to_nvblox_bridge mid360_bridge_node_exe --help
```

## 三、配置匹配

### 关键：参数必须精确匹配！

#### Bridge配置 (`mid360_bridge.yaml`)
```yaml
virtual_lidar_width: 1800
virtual_lidar_height: 32
min_elevation_deg: -7.0
max_elevation_deg: 52.0
```

#### nvblox配置 (`nvblox_base.yaml`)
```yaml
# 必须与bridge完全一致！
lidar_width: 1800
lidar_height: 32
use_non_equal_vertical_fov_lidar_params: true
min_angle_below_zero_elevation_rad: 0.122173  # 7° * π/180
max_angle_above_zero_elevation_rad: 0.907571  # 52° * π/180
lidar_min_valid_range_m: 0.5
lidar_max_valid_range_m: 30.0
```

**计算转换**:
```bash
# 度数转弧度
echo "scale=6; 7 * 3.14159265359 / 180" | bc   # = 0.122173
echo "scale=6; 52 * 3.14159265359 / 180" | bc  # = 0.907571
```

## 四、启动流程

### 方案A: 分步启动（调试推荐）

```bash
# Terminal 1: Mid360驱动
cd ~/ros2_ws
source install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py

# 等待Mid360连接成功，看到点云发布...

# Terminal 2: Bridge节点
ros2 launch mid360_to_nvblox_bridge mid360_bridge.launch.py

# 等待看到 "Mid360 Bridge Node initialized"

# Terminal 3: nvblox
ros2 launch nvblox_examples_bringup nvblox.launch.py \
    setup_for_lidar:=True

# Terminal 4: 诊断工具（可选）
ros2 run mid360_to_nvblox_bridge bridge_diagnostics.py
```

### 方案B: 一键启动（生产环境）

创建统一launch文件 `full_system.launch.py`:

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    return LaunchDescription([
        # 1. Mid360驱动
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('livox_ros_driver2'),
                    'launch', 'msg_MID360_launch.py'
                ])
            ])
        ),
        # 2. Bridge
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('mid360_to_nvblox_bridge'),
                    'launch', 'mid360_bridge.launch.py'
                ])
            ])
        ),
        # 3. nvblox
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('nvblox_examples_bringup'),
                    'launch', 'nvblox.launch.py'
                ])
            ]),
            launch_arguments={'setup_for_lidar': 'True'}.items()
        ),
    ])
```

启动：
```bash
ros2 launch full_system.launch.py
```

## 五、验证与调试

### 5.1 检查话题

```bash
# 列出所有点云话题
ros2 topic list | grep -i point

# 应该看到:
# /livox/lidar                        # Mid360原始
# /lidar/pointcloud_structured        # Bridge输出
# /nvblox_node/...                    # nvblox相关

# 检查消息频率
ros2 topic hz /livox/lidar
ros2 topic hz /lidar/pointcloud_structured

# 检查消息结构
ros2 topic echo /lidar/pointcloud_structured --once | grep -E "height|width"
# 应该显示: height: 32, width: 1800
```

### 5.2 性能监控

```bash
# CPU使用率
top -p $(pgrep mid360_bridge)

# 内存使用
ros2 run ros2_tools memory_monitor

# 完整诊断
ros2 run mid360_to_nvblox_bridge bridge_diagnostics.py
```

### 5.3 可视化验证

```bash
# 启动RViz2
rviz2

# 添加以下显示:
# 1. PointCloud2: /livox/lidar (颜色: 白色)
# 2. PointCloud2: /lidar/pointcloud_structured (颜色: 绿色)
# 3. Mesh: /nvblox_node/mesh (如有)
# 4. Map: /nvblox_node/map (如有)

# 对比原始与转换后的点云:
# - 转换后的应该呈现规则网格结构
# - 密度应该与原始接近（>70%）
# - 无明显几何失真
```

## 六、常见问题排查

### 问题1: Bridge节点崩溃

**症状**: 节点启动后立即退出

**排查**:
```bash
# 查看日志
ros2 run mid360_to_nvblox_bridge mid360_bridge_node_exe --ros-args --log-level debug

# 检查配置文件路径
ls ~/ros2_ws/install/mid360_to_nvblox_bridge/share/mid360_to_nvblox_bridge/config/

# 验证YAML语法
python3 -c "import yaml; yaml.safe_load(open('config/mid360_bridge.yaml'))"
```

**解决**:
- 检查配置文件格式
- 确保参数在合理范围内
- 查看详细错误日志

### 问题2: nvblox拒绝点云

**症状**: nvblox日志显示 "LiDAR intrinsics are inconsistent"

**排查**:
```bash
# 比较参数
ros2 param get /mid360_bridge virtual_lidar_width
ros2 param get /mid360_bridge virtual_lidar_height

ros2 param get /nvblox_node lidar_width
ros2 param get /nvblox_node lidar_height

# 它们必须完全一致！
```

**解决**:
1. 修改nvblox配置文件匹配bridge输出
2. 或修改bridge配置匹配nvblox期望
3. 重启两个节点

### 问题3: 点云密度太低

**症状**: 输出点云稀疏，valid_ratio < 50%

**原因**:
- FOV配置不匹配
- 分辨率设置过高
- Range限制过严

**解决**:
```yaml
# 调整bridge配置
min_elevation_deg: -10.0      # 扩大垂直FOV
max_elevation_deg: 55.0
max_range_m: 50.0             # 增加范围
enable_hole_filling: true     # 启用填充
max_hole_fill_iterations: 3   # 增加迭代
```

### 问题4: 高延迟

**症状**: 处理延迟 > 20ms

**优化**:
```yaml
# 降低分辨率
virtual_lidar_width: 900      # 从1800降到900
virtual_lidar_height: 16      # 从32降到16
enable_hole_filling: false    # 禁用填充
```

### 问题5: 建图质量差

**症状**: nvblox地图有明显失真

**排查**:
1. 检查Mid360原始数据质量
2. 验证TF树完整性
3. 对比原始和转换后点云

**解决**:
```yaml
# 提高质量（牺牲性能）
virtual_lidar_width: 3600
virtual_lidar_height: 64
aggregation_method: "median"  # 更鲁棒
```

## 七、性能调优

### 场景1: 室内导航（高精度）

```yaml
# mid360_bridge.yaml
virtual_lidar_width: 3600
virtual_lidar_height: 64
min_range_m: 0.3
max_range_m: 20.0
enable_hole_filling: true
max_hole_fill_iterations: 3
aggregation_method: "mean"

# nvblox_base.yaml
voxel_size: 0.025              # 2.5cm精度
integrate_lidar_rate_hz: 20.0
update_esdf_rate_hz: 10.0
```

### 场景2: 户外探索（大范围）

```yaml
# mid360_bridge.yaml
virtual_lidar_width: 1800
virtual_lidar_height: 32
min_range_m: 1.0
max_range_m: 50.0
enable_hole_filling: true
max_hole_fill_iterations: 2
aggregation_method: "min"      # 保守，检测障碍

# nvblox_base.yaml
voxel_size: 0.10               # 10cm精度
integrate_lidar_rate_hz: 15.0
map_clearing_radius_m: 15.0
```

### 场景3: 高速移动（实时性）

```yaml
# mid360_bridge.yaml
virtual_lidar_width: 900
virtual_lidar_height: 16
min_range_m: 0.5
max_range_m: 30.0
enable_hole_filling: false     # 禁用以降低延迟
aggregation_method: "min"

# nvblox_base.yaml
voxel_size: 0.05
integrate_lidar_rate_hz: 30.0
update_mesh_rate_hz: 2.0       # 降低mesh更新频率
```

## 八、自动化脚本

### 一键诊断脚本

创建 `diagnose.sh`:

```bash
#!/bin/bash
echo "=== Mid360 + nvblox System Diagnostics ==="
echo ""

echo "1. Checking ROS2 environment..."
if [ -z "$ROS_DISTRO" ]; then
    echo "❌ ROS2 not sourced!"
    exit 1
fi
echo "✓ ROS2 $ROS_DISTRO"

echo ""
echo "2. Checking packages..."
ros2 pkg list | grep -q mid360_to_nvblox_bridge && echo "✓ Bridge package" || echo "❌ Bridge not found"
ros2 pkg list | grep -q nvblox && echo "✓ nvblox package" || echo "❌ nvblox not found"
ros2 pkg list | grep -q livox_ros_driver2 && echo "✓ Livox driver" || echo "❌ Livox not found"

echo ""
echo "3. Checking topics..."
timeout 5 ros2 topic list 2>/dev/null | grep -q "/livox/lidar" && echo "✓ Mid360 publishing" || echo "⚠ Mid360 not active"
timeout 5 ros2 topic list 2>/dev/null | grep -q "/lidar/pointcloud_structured" && echo "✓ Bridge publishing" || echo "⚠ Bridge not active"

echo ""
echo "4. Checking nodes..."
ros2 node list 2>/dev/null | grep -q "mid360_bridge" && echo "✓ Bridge node running" || echo "⚠ Bridge not running"
ros2 node list 2>/dev/null | grep -q "nvblox_node" && echo "✓ nvblox node running" || echo "⚠ nvblox not running"

echo ""
echo "5. Parameter check..."
if ros2 node list 2>/dev/null | grep -q "mid360_bridge"; then
    WIDTH=$(ros2 param get /mid360_bridge virtual_lidar_width 2>/dev/null | awk '{print $2}')
    HEIGHT=$(ros2 param get /mid360_bridge virtual_lidar_height 2>/dev/null | awk '{print $2}')
    echo "  Bridge: ${WIDTH}x${HEIGHT}"
fi

if ros2 node list 2>/dev/null | grep -q "nvblox_node"; then
    WIDTH_NV=$(ros2 param get /nvblox_node lidar_width 2>/dev/null | awk '{print $2}')
    HEIGHT_NV=$(ros2 param get /nvblox_node lidar_height 2>/dev/null | awk '{print $2}')
    echo "  nvblox: ${WIDTH_NV}x${HEIGHT_NV}"
    
    if [ "$WIDTH" == "$WIDTH_NV" ] && [ "$HEIGHT" == "$HEIGHT_NV" ]; then
        echo "  ✓ Parameters match!"
    else
        echo "  ❌ Parameter mismatch!"
    fi
fi

echo ""
echo "=== Diagnostics Complete ==="
```

使用:
```bash
chmod +x diagnose.sh
./diagnose.sh
```

## 九、生产环境检查清单

部署前检查：

- [ ] 所有软件包已安装并编译
- [ ] Bridge和nvblox参数完全匹配
- [ ] Mid360固件已更新到最新版本
- [ ] TF树正确配置（lidar → base_link → odom）
- [ ] 系统资源充足（CPU < 60%, RAM < 80%）
- [ ] 诊断工具显示valid_ratio > 70%
- [ ] RViz2可视化验证无失真
- [ ] 建图质量满足应用需求
- [ ] 延迟测试 < 15ms
- [ ] 长时间稳定性测试（>1小时）

## 十、获取支持

遇到问题？

1. **查看日志**: `ros2 run ... --ros-args --log-level debug`
2. **运行诊断**: `./diagnose.sh`
3. **检查文档**: `README.md`, `TECHNICAL_DESIGN.md`
4. **社区支持**: GitHub Issues
5. **专业支持**: 联系维护团队

---

**部署成功标志**:
- ✅ 所有节点正常运行无错误
- ✅ 点云有效率 > 70%
- ✅ nvblox正常建图
- ✅ 处理延迟 < 10ms
- ✅ 长时间稳定运行

祝部署顺利！🎉
