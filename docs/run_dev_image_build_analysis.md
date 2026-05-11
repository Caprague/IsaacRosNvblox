# run_dev.sh 镜像构建流程分析

## 整体架构

`run_dev.sh` 是 Isaac ROS 开发环境的入口脚本，位于 `src/isaac_ros_common/scripts/run_dev.sh`，负责 **构建 Docker 镜像** 并 **启动容器**。核心构建逻辑委托给 `build_image_layers.sh`。

---

## 1. 入口: run_dev.sh

**关键步骤：**

1. **读取配置文件** `.isaac_ros_common-config`，当前配置为：
   ```
   CONFIG_IMAGE_KEY=ros2_humble.realsense
   ```

2. **确定 IMAGE_KEY**：默认 `ros2_humble`，被配置覆盖为 `ros2_humble.realsense`

3. **拼接 BASE_IMAGE_KEY**：`{平台}.{IMAGE_KEY}`
   - 在 aarch64 平台上：`aarch64.ros2_humble.realsense`

4. **调用构建脚本**（run_dev.sh 第205行）：
   ```bash
   $ROOT/build_image_layers.sh --image_key "aarch64.ros2_humble.realsense" --image_name "isaac_ros_dev-aarch64"
   ```

5. **启动容器**：用构建好的镜像 `docker run` 启动，挂载工作空间到 `/workspaces/isaac_ros-dev`

---

## 2. 核心: build_image_layers.sh — 分层构建

**核心机制是将 image_key 用 `.` 分割，然后逐段匹配 Dockerfile 文件。**

对于 `aarch64.ros2_humble.realsense`，分割为：`["aarch64", "ros2_humble", "realsense"]`

### 匹配算法（从长到短的尾部匹配）

1. 第一轮尝试匹配：`Dockerfile.aarch64.ros2_humble.realsense` -> 不存在
2. 接着尝试：`Dockerfile.ros2_humble.realsense` -> 不存在
3. 接着尝试：`Dockerfile.realsense` -> **找到！**
4. 剩余未匹配部分 `["aarch64", "ros2_humble"]`，继续递归：
   - 尝试：`Dockerfile.aarch64.ros2_humble` -> 不存在
   - 尝试：`Dockerfile.ros2_humble` -> **找到！**
   - 剩余：`["aarch64"]`
   - 尝试：`Dockerfile.aarch64` -> **找到！**

### 最终解析出 3 个 Dockerfile，按构建顺序（从底层到顶层）

| 层级 | Dockerfile | 作用 |
|------|-----------|------|
| 1 (基础层) | `Dockerfile.aarch64` | CUDA/TensorRT/PyTorch/Triton/VPI 等 GPU 基础环境 |
| 2 (中间层) | `Dockerfile.ros2_humble` | ROS 2 Humble + MoveIt2 + Nav2 等 ROS 生态 |
| 3 (顶层) | `Dockerfile.realsense` | RealSense SDK + realsense-ros 驱动 |

### 构建链路

```
Dockerfile.aarch64  ->  (base: nvcr.io/nvidia/12.6.11-devel:...-aarch64-ubuntu22.04)
      | (作为 BASE_IMAGE)
      v
Dockerfile.ros2_humble  ->  (base: 上一层产出的镜像)
      | (作为 BASE_IMAGE)
      v
Dockerfile.realsense  ->  (base: 上一层产出的镜像) -> 最终镜像: isaac_ros_dev-aarch64
```

---

## 3. 各 Dockerfile 内容概览

### Dockerfile.aarch64（基础层）

- **基础镜像**：`nvcr.io/nvidia/12.6.11-devel:12.6.11-devel-aarch64-ubuntu22.04`
- **主要安装内容**：
  - CUDA 12.6 全套工具链
  - TensorRT 10.3.0.30
  - VPI 3.2.4
  - PyTorch 2.5 (NV CUDA Jetson 版)
  - Triton Server 2.49 (iGPU 版)
  - cuDSS 0.3.0.9
  - CV-CUDA 0.5.0
  - Boost 1.80
  - FFmpeg 4.4.2
  - Protobuf v5.26.0
  - Ceres Solver (CUDA加速)
  - Python 3.10 及大量科学计算包 (numpy, scipy, scikit-learn 等)
  - Node.js 18, Yarn

### Dockerfile.ros2_humble（中间层）

- **基础镜像**：上一层产出
- **主要安装内容**：
  - ROS 2 Humble (ros-humble-ros-base)
  - Navigation2 全套 (nav2-bringup, nav2-mppi-controller 等)
  - MoveIt2 全套 (moveit, moveit-servo, moveit-task-constructor 等)
  - image_pipeline (带 backport 修复)
  - rclcpp (带 multithreadedexecutor 修复)
  - colcon 构建工具
  - rosdep 依赖管理
  - MCAP CLI v0.0.51
  - Foxglove Bridge
  - RViz2, rqt 工具集
  - SLAM Toolbox
  - UR 机器人驱动

### Dockerfile.realsense（顶层）

- **基础镜像**：上一层产出
- **主要安装内容**：
  - librealsense v2.55.1（源码编译）
  - realsense-ros 4.51.1 (NVIDIA Isaac ROS fork)
  - USB hotplug udev 规则
- **注意**：该文件内配置了代理 (127.0.0.1:7890)

---

## 4. 预构建镜像加速机制

`build_image_layers.sh` 在本地构建前会检查 NVIDIA 远程仓库 (`nvcr.io/nvidia/isaac/ros`) 是否有与当前 Dockerfile 内容 MD5 哈希匹配的预构建镜像：

1. 对每层 Dockerfile 计算 MD5 哈希
2. 拼接为镜像 tag：`nvcr.io/nvidia/isaac/ros:{image_key}_{md5hash}`
3. 通过 `docker manifest inspect` 检查远端是否存在
4. 如果找到，直接 `docker pull` 跳过本地构建

---

## 5. 构建参数

| 参数 | 值 | 说明 |
|------|------|------|
| `USERNAME` | `admin` | 容器内非 root 用户 |
| `PLATFORM` | `arm64` | 架构标识 (aarch64 映射为 arm64) |
| `HAS_GPU` | `true` | GPU 检测（仅 x86_64 生效） |
| `DOCKER_BUILDKIT` | `1` | 启用 BuildKit 加速构建 |

---

## 6. 容器运行配置

最终容器启动参数（aarch64 平台）：

- `--privileged` — 特权模式
- `--network host` — 主机网络
- `--runtime nvidia` — NVIDIA 容器运行时
- `--ipc=host` — 共享 IPC 命名空间
- `--pid=host` — 共享 PID 命名空间
- 挂载 X11 socket、tegrastats、VPI3、Jetson Multimedia API 等主机资源
- 工作空间挂载：`$ISAAC_ROS_DEV_DIR:/workspaces/isaac_ros-dev`
- 入口点：`/usr/local/bin/scripts/workspace-entrypoint.sh`

---

## 7. 配置文件搜索路径

Docker 搜索目录按优先级：
1. `~/.isaac_ros_common-config` 中 `CONFIG_DOCKER_SEARCH_DIRS` 指定的目录
2. `scripts/.isaac_ros_common-config` 中指定的目录
3. 默认：`src/isaac_ros_common/docker/`

---

## 8. 文件索引

```
src/isaac_ros_common/
├── scripts/
│   ├── run_dev.sh                    # 入口脚本
│   ├── build_image_layers.sh         # 分层构建核心脚本
│   ├── build_base_image.sh           # 基础镜像构建
│   ├── .isaac_ros_common-config      # 默认配置
│   └── utils/
│       └── print_color.sh            # 彩色输出工具
├── docker/
│   ├── Dockerfile.aarch64            # 基础层 (GPU/CUDA)
│   ├── Dockerfile.x86_64            # x86_64 基础层
│   ├── Dockerfile.ros2_humble        # ROS 2 层
│   ├── Dockerfile.realsense          # RealSense 层
│   ├── Dockerfile.base               # 通用基础层
│   ├── scripts/                      # 容器内脚本 (entrypoint等)
│   ├── patches/                      # 源码补丁
│   ├── rosdep/                       # rosdep 配置
│   └── middleware_profiles/          # DDS 中间件配置
```
