# Isaac ROS 开发工作区

## 项目概述

这是一个基于 Isaac ROS 的 ROS 2 开发工作区，包含多个用于机器人感知和导航的软件包。工作区位于 `/home/nvidia/IsaacRos/isaac_ros-dev/src`，运行在 Linux 5.15.148-tegra 操作系统上，目标平台为 Jetson Orin。

### 主要组件

1. **isaac_ros_common** - Isaac ROS 通用工具包
   - 提供开发脚本和 Dockerfiles
   - 包含通用工具和实用函数
   - 版本：3.2.5
   - 许可证：NVIDIA Isaac ROS Software License

2. **isaac_ros_nvblox** - 3D 场景重建和导航包
   - 基于 nvblox 的 ROS 2 集成
   - 实时 3D 场景重建和 2D 代价地图生成
   - 支持 Nav2 导航集成
   - 支持 GPU 加速计算
   - 版本：3.2.5
   - 许可证：Apache-2.0

3. **odom_horizontal_transform** - IMU 水平对齐节点
   - 自定义包，用于 IMU 水平对齐
   - 计算并发布从 odom_horizontal 到 map 的变换
   - 版本：0.0.0

### 技术栈

- **ROS 2**: Humble
- **编程语言**: C++17, Python
- **构建系统**: ament_cmake, ament_cmake_auto
- **CUDA**: 支持 GPU 加速（Jetson Orin 架构 87）
- **依赖库**:
  - Eigen3
  - OpenCV (cv_bridge)
  - tf2_ros, tf2_eigen
  - sensor_msgs, geometry_msgs, nav_msgs
  - NVIDIA VPI (Vision Programming Interface)
  - nvblox 核心库

## 构建和运行

### 环境设置

使用 Docker 容器进行开发：

```bash
# 进入 isaac_ros_common/scripts 目录
cd /home/nvidia/IsaacRos/isaac_ros-dev/src/isaac_ros_common/scripts

# 运行开发环境（默认映射到 ~/workspaces/isaac_ros-dev）
./run_dev.sh

# 或指定工作区路径
./run_dev.sh -d /path/to/workspace
```

### 构建项目

在 Docker 容器内或已配置的环境中：

```bash
# 进入工作区根目录
cd /home/nvidia/IsaacRos/isaac_ros-dev

# 构建所有包
colcon build

# 构建特定包
colcon build --packages-select odom_horizontal_transform

# 使用本地 CUDA 架构构建以加快速度
colcon build --cmake-args "-DUSE_NATIVE_CUDA_ARCHITECTURE=1"

# Release 模式构建
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### 运行节点

```bash
# 加载工作区环境
source /home/nvidia/IsaacRos/isaac_ros-dev/install/setup.bash

# 运行 IMU 水平对齐测试
ros2 launch odom_horizontal_transform test_imu_horizontal_align.launch.py

# 运行 nvblox 节点（需要先配置参数）
ros2 launch nvblox_ros <launch_file>
```

### 测试

```bash
# 运行所有测试
colcon test

# 运行特定包的测试
colcon test --packages-select odom_horizontal_transform

# 运行 Python 测试（nvblox_test）
cd /home/nvidia/IsaacRos/isaac_ros-dev/src/isaac_ros_nvblox/nvblox_test
pytest

# 查看测试结果
colcon test-result --verbose
```

## 开发约定

### 包结构

每个 ROS 2 包应包含：

- `package.xml` - 包元数据和依赖
- `CMakeLists.txt` - CMake 构建配置
- `src/` - 源代码目录
- `include/` - 头文件目录（C++）
- `launch/` - ROS 2 launch 文件
- `config/` - 配置文件（可选）
- `test/` - 测试文件

### 代码风格

- **C++**: 使用 C++17 标准
  - 编译选项：`-Wall -Wextra -Wpedantic`
  - 使用 `ament_cmake_auto` 简化构建配置
  - 遵循 ROS 2 编码规范

- **Python**: 使用 Python 3
  - 遵循 PEP 8 编码规范
  - 使用 pytest 进行测试

### 许可证

- Isaac ROS 包使用 NVIDIA Isaac ROS Software License 或 Apache-2.0
- 自定义包应明确声明许可证

### 版本控制

- 使用 Git 进行版本控制
- 子模块：isaac_ros_nvblox 包含 nvblox_core 子模块
- 遵循 NVIDIA Isaac ROS 的版本规范（当前 3.2.5）

### 依赖管理

- 使用 `package.xml` 声明运行时和构建时依赖
- 使用 `find_package()` 在 CMakeLists.txt 中查找依赖
- 使用 `ament_target_dependencies()` 链接 ROS 2 依赖

### CUDA 集成

- Jetson Orin 默认使用 CUDA 架构 87
- 支持多架构构建以兼容不同 GPU
- 使用 `CMAKE_CUDA_ARCHITECTURES` 控制目标架构

### 消息和接口

- 自定义消息定义在 `*_interfaces` 包中
- 使用标准 ROS 2 消息类型（sensor_msgs, geometry_msgs 等）
- TF2 用于坐标变换

## 关键文件说明

### isaac_ros_common

- `scripts/run_dev.sh` - 启动开发环境
- `scripts/build_base_image.sh` - 构建 Docker 基础镜像
- `scripts/build_image_layers.sh` - 构建镜像层
- `isaac_ros_common/src/vpi_utilities.cpp` - VPI 工具函数

### isaac_ros_nvblox

- `nvblox_ros/src/lib/nvblox_node.cpp` - 主要 nvblox 节点
- `nvblox_ros/src/lib/fuser_node.cpp` - 融合节点
- `nvblox_nav2/` - Nav2 代价地图插件
- `nvblox_msgs/` - 自定义消息定义
- `nvblox_rviz_plugin/` - RViz 可视化插件

### odom_horizontal_transform

- `src/imu_horizontal_align.cpp` - IMU 水平对齐实现
- `launch/test_imu_horizontal_align.launch.py` - 测试启动文件
- `include/odom_horizontal_transform/` - 头文件目录

## 性能考虑

- nvblox 在 Jetson Orin 上的性能指标（0.05m 体素大小）：
  - TSDF 融合：0.8 ms
  - 颜色融合：1.1 ms
  - 网格生成：2.3 ms
  - ESDF 计算：1.7 ms

- 使用 GPU 加速以实现实时性能
- 考虑使用 `USE_NATIVE_CUDA_ARCHITECTURE` 加快构建速度

## 故障排除

- 查看 Isaac ROS 文档的故障排除部分：https://nvidia-isaac-ros.github.io/troubleshooting/index.html
- 常见问题包括：
  - Isaac Sim 集成问题
  - RealSense 相机问题
  - ROS 通信问题

## 资源

- 官方文档：https://nvidia-isaac-ros.github.io/
- Isaac ROS Nvblox 文档：https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/index.html
- NVIDIA 开发者网站：https://developer.nvidia.com/isaac-ros-gems/