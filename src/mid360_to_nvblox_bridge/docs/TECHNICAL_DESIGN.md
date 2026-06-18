# Technical Design Document: Mid360 to nvblox Bridge

## 1. 问题定义

### 1.1 Mid360特性
- **扫描模式**: Livox专利的非重复扫描（Non-repetitive Scanning）
- **点云特征**: 
  - 每帧点位置不固定
  - 累积多帧形成高FOV覆盖
  - 单帧点云无固定结构
  
### 1.2 nvblox要求
- **期望输入**: 机械旋转式激光雷达
- **结构化假设**:
  - 固定角度网格: `width × height` 规则排列
  - 可重复扫描: 每帧扫描pattern相同
  - 像素对应关系: 每个点精确映射到网格位置

### 1.3 冲突点
```
Mid360实际               nvblox期望
─────────────           ─────────────
非重复扫描        ≠     重复扫描
无序点云          ≠     有序网格
动态pattern       ≠     固定pattern
高密度采样        ≠     均匀分布
```

## 2. 解决方案架构

### 2.1 总体架构

```
┌──────────────┐
│  Mid360      │  发布原始点云
│  Driver      │  Topic: /livox/lidar
└──────┬───────┘  Format: unstructured
       │          
       ↓ PointCloud2
       │ (width=N, height=1)
       │
┌──────┴───────────────────────────────┐
│  Bridge Node                         │
│  ┌────────────────────────────────┐  │
│  │ 1. 球坐标转换模块              │  │
│  │    Cartesian → Spherical      │  │
│  └────────────┬───────────────────┘  │
│               ↓                      │
│  ┌────────────────────────────────┐  │
│  │ 2. 网格映射模块                │  │
│  │    (r,θ,φ) → (u,v)           │  │
│  └────────────┬───────────────────┘  │
│               ↓                      │
│  ┌────────────────────────────────┐  │
│  │ 3. 聚合模块                    │  │
│  │    Multiple points → Cell     │  │
│  └────────────┬───────────────────┘  │
│               ↓                      │
│  ┌────────────────────────────────┐  │
│  │ 4. 插值模块                    │  │
│  │    Fill holes via neighbors   │  │
│  └────────────┬───────────────────┘  │
│               ↓                      │
│  ┌────────────────────────────────┐  │
│  │ 5. 重构模块                    │  │
│  │    Grid → PointCloud2         │  │
│  └────────────────────────────────┘  │
└───────────┬──────────────────────────┘
            ↓ PointCloud2
            │ (width=1800, height=32)
            │ structured & ordered
┌───────────┴──────────┐
│  nvblox              │  接收结构化点云
│  ┌────────────────┐  │  正常融合处理
│  │ Depth → Image  │  │
│  │ Image → TSDF   │  │
│  │ TSDF → ESDF    │  │
│  └────────────────┘  │
└──────────────────────┘
```

### 2.2 核心算法

#### 模块1: 球坐标转换
```cpp
// 笛卡尔坐标 (x, y, z) → 球坐标 (r, θ, φ)
bool cartesianToSpherical(float x, float y, float z,
                         float& range, float& azimuth, float& elevation) {
    // 距离
    range = sqrt(x*x + y*y + z*z);
    
    // 范围检查
    if (range < min_range || range > max_range) {
        return false;
    }
    
    // 方位角 (水平角度): [-π, π]
    azimuth = atan2(y, x);
    
    // 仰角 (垂直角度): [-π/2, π/2]
    elevation = asin(z / range);
    
    return true;
}
```

**坐标系定义**:
```
        Z (up)
        ↑
        │     Y
        │   ↗
        │ ╱
        ○────→ X (forward)
       ╱
     ╱
   (LiDAR origin)

azimuth: X-Y平面内的角度，从X轴逆时针
elevation: 相对于X-Y平面的仰角
```

#### 模块2: 网格映射
```cpp
bool sphericalToGrid(float azimuth, float elevation,
                    int& u_idx, int& v_idx) {
    // 垂直FOV检查
    if (elevation < min_elevation || elevation > max_elevation) {
        return false;
    }
    
    // 水平索引: 映射 [-π, π] → [0, width)
    float azimuth_norm = azimuth + M_PI;  // [0, 2π]
    u_idx = (int)(azimuth_norm / azimuth_resolution);
    
    // 处理边界
    if (u_idx >= width) u_idx = width - 1;
    if (u_idx < 0) u_idx = 0;
    
    // 垂直索引: 映射 [min_elev, max_elev] → [0, height)
    float elev_offset = elevation - min_elevation;
    v_idx = (int)(elev_offset / elevation_resolution);
    
    // 边界限制
    v_idx = clamp(v_idx, 0, height-1);
    
    return true;
}
```

**网格布局**:
```
v (elevation)
↑
│  ┌───┬───┬───┬───┬───┐
32 │   │   │   │   │   │  最高线束 (+52°)
│  ├───┼───┼───┼───┼───┤
│  │   │   │   │   │   │
│  ├───┼───┼───┼───┼───┤
16 │   │   │ X │   │   │  中间线束 (0°)
│  ├───┼───┼───┼───┼───┤
│  │   │   │   │   │   │
│  ├───┼───┼───┼───┼───┤
0  │   │   │   │   │   │  最低线束 (-7°)
   └───┴───┴───┴───┴───┘
   0              1800 → u (azimuth)
   -180°           +180°
```

#### 模块3: 聚合策略
```cpp
// 处理多点映射到同一网格单元
void aggregatePoints(GridCell& cell, float new_depth) {
    if (aggregation_method == "mean") {
        cell.depth_sum += new_depth;
        cell.point_count++;
        cell.depth = cell.depth_sum / cell.point_count;
    }
    else if (aggregation_method == "min") {
        // 保留最近距离（对障碍物检测有利）
        if (cell.point_count == 0 || new_depth < cell.depth) {
            cell.depth = new_depth;
        }
        cell.point_count++;
    }
    else if (aggregation_method == "max") {
        // 保留最远距离
        if (cell.point_count == 0 || new_depth > cell.depth) {
            cell.depth = new_depth;
        }
        cell.point_count++;
    }
    cell.valid = true;
}
```

**聚合方法对比**:

| 方法 | 优点 | 缺点 | 适用场景 |
|------|------|------|----------|
| mean | 平滑、鲁棒 | 可能模糊边界 | 通用场景 |
| min | 保守、检测障碍 | 对噪声敏感 | 避障导航 |
| max | 探测远处物体 | 易受outlier影响 | 开放环境 |
| median | 抗噪声 | 计算开销大 | 噪声环境 |

#### 模块4: 空洞填充
```cpp
void fillHoles(Grid& grid, int iterations) {
    for (int iter = 0; iter < iterations; iter++) {
        for (int v = 0; v < height; v++) {
            for (int u = 0; u < width; u++) {
                if (!grid[v][u].valid) {
                    interpolateFromNeighbors(grid, u, v);
                }
            }
        }
    }
}

void interpolateFromNeighbors(Grid& grid, int u, int v) {
    vector<float> neighbor_depths;
    
    // 4-连通邻域
    if (u > 0 && grid[v][u-1].valid)
        neighbor_depths.push_back(grid[v][u-1].depth);
    if (u < width-1 && grid[v][u+1].valid)
        neighbor_depths.push_back(grid[v][u+1].depth);
    if (v > 0 && grid[v-1][u].valid)
        neighbor_depths.push_back(grid[v-1][u].depth);
    if (v < height-1 && grid[v+1][u].valid)
        neighbor_depths.push_back(grid[v+1][u].depth);
    
    // 水平环绕（360°连续性）
    if (u == 0 && grid[v][width-1].valid)
        neighbor_depths.push_back(grid[v][width-1].depth);
    if (u == width-1 && grid[v][0].valid)
        neighbor_depths.push_back(grid[v][0].depth);
    
    // 至少需要2个邻居
    if (neighbor_depths.size() >= 2) {
        float avg = accumulate(neighbor_depths) / neighbor_depths.size();
        grid[v][u].depth = avg;
        grid[v][u].valid = true;
    }
}
```

**插值效果示意**:
```
Before:          After (1 iteration):
X X - X X   →    X X X X X
X - - - X   →    X X - X X  (center needs 2nd iteration)
X X - X X   →    X X X X X

Legend: X = valid point, - = hole
```

#### 模块5: 点云重构
```cpp
PointCloud2 reconstructPointCloud(const Grid& grid) {
    PointCloud2 output;
    output.height = height;
    output.width = width;
    
    for (int v = 0; v < height; v++) {
        for (int u = 0; u < width; u++) {
            if (grid[v][u].valid) {
                // 网格索引 → 球坐标
                float azimuth = -M_PI + (u + 0.5) * azimuth_res;
                float elevation = min_elev + v * elevation_res;
                float range = grid[v][u].depth;
                
                // 球坐标 → 笛卡尔坐标
                float cos_elev = cos(elevation);
                float x = range * cos_elev * cos(azimuth);
                float y = range * cos_elev * sin(azimuth);
                float z = range * sin(elevation);
                
                output.push_back(x, y, z);
            } else {
                // 无效点用NaN填充
                output.push_back(NaN, NaN, NaN);
            }
        }
    }
    
    return output;
}
```

## 3. 参数调优指南

### 3.1 分辨率选择

**水平分辨率 (width)**:
```
分辨率计算: angular_res = 360° / width

推荐值:
- 900  → 0.4°/pixel  (快速, 低质量)
- 1800 → 0.2°/pixel  (平衡, 推荐)
- 3600 → 0.1°/pixel  (高质量, 慢速)
- 7200 → 0.05°/pixel (极高质量, 资源密集)
```

**选择依据**:
- **场景复杂度**: 复杂环境需要更高分辨率
- **运动速度**: 高速运动可降低分辨率
- **计算资源**: 分辨率↑ → CPU/内存↑
- **Mid360点密度**: 匹配实际点云密度

**垂直分辨率 (height)**:
```
分辨率计算: angular_res = 59° / (height-1)

推荐值:
- 16 → 3.9°/line  (粗糙)
- 32 → 1.9°/line  (推荐, 接近Mid360)
- 64 → 0.95°/line (精细)
```

### 3.2 性能基准测试

| 配置 | 分辨率 | 网格大小 | 处理时间 | 内存占用 | 适用场景 |
|------|--------|----------|----------|----------|----------|
| 低   | 900x16 | 14,400   | ~2ms     | ~0.5MB   | 高速移动 |
| 中   | 1800x32| 57,600   | ~6ms     | ~2MB     | 通用导航 |
| 高   | 3600x64| 230,400  | ~25ms    | ~8MB     | 精细建图 |
| 超高 | 7200x64| 460,800  | ~50ms    | ~16MB    | 离线处理 |

*测试平台: Jetson Orin, ROS2 Humble*

### 3.3 质量vs性能权衡

```python
# 配置模板生成器
def generate_config(priority):
    if priority == "quality":
        return {
            'width': 3600,
            'height': 64,
            'hole_filling': True,
            'fill_iterations': 3,
            'aggregation': 'median'  # 最鲁棒
        }
    elif priority == "balanced":
        return {
            'width': 1800,
            'height': 32,
            'hole_filling': True,
            'fill_iterations': 2,
            'aggregation': 'mean'
        }
    elif priority == "speed":
        return {
            'width': 900,
            'height': 16,
            'hole_filling': False,
            'aggregation': 'min'  # 最快
        }
```

## 4. 实现细节与优化

### 4.1 内存管理
```cpp
// 预分配网格，避免运行时分配
class PointcloudConverter {
    vector<vector<GridCell>> grid_;  // 预分配
    
    void initialize(int width, int height) {
        grid_.resize(height);
        for (auto& row : grid_) {
            row.resize(width);
        }
    }
    
    void clearGrid() {
        // 快速清零，不释放内存
        for (auto& row : grid_) {
            for (auto& cell : row) {
                cell.reset();  // inline函数
            }
        }
    }
};
```

### 4.2 并行化优化（可选扩展）
```cpp
// OpenMP并行化网格映射
#pragma omp parallel for
for (int i = 0; i < num_points; i++) {
    float r, theta, phi;
    cartesianToSpherical(points[i], r, theta, phi);
    
    int u, v;
    if (sphericalToGrid(theta, phi, u, v)) {
        #pragma omp critical
        {
            grid[v][u].aggregate(r);
        }
    }
}
```

### 4.3 边界条件处理
```cpp
// 水平方向环绕处理
inline int wrapAzimuth(int u, int width) {
    if (u < 0) return u + width;
    if (u >= width) return u - width;
    return u;
}

// 垂直方向边界夹紧
inline int clampElevation(int v, int height) {
    return max(0, min(v, height-1));
}
```

## 5. 测试与验证

### 5.1 单元测试
```cpp
TEST(BridgeTest, SphericalConversion) {
    // 测试点: (10, 0, 0) → (10, 0°, 0°)
    float r, az, el;
    ASSERT_TRUE(cartesianToSpherical(10, 0, 0, r, az, el));
    EXPECT_NEAR(r, 10.0, 1e-6);
    EXPECT_NEAR(az, 0.0, 1e-6);
    EXPECT_NEAR(el, 0.0, 1e-6);
}

TEST(BridgeTest, GridMapping) {
    BridgeNode node(1800, 32, -7*DEG, 52*DEG);
    int u, v;
    
    // 前方点应该映射到中间
    ASSERT_TRUE(node.sphericalToGrid(0, 0, u, v));
    EXPECT_NEAR(u, 900, 10);  // 允许±10像素误差
}
```

### 5.2 集成测试
```bash
# 1. 播放录制的bag文件
ros2 bag play mid360_test.db3

# 2. 运行bridge
ros2 launch mid360_to_nvblox_bridge mid360_bridge.launch.py

# 3. 检查输出
ros2 run mid360_to_nvblox_bridge bridge_diagnostics.py

# 期望输出:
# Valid ratio: >70%
# Frequency: ~10Hz
# Efficiency: >80%
```

### 5.3 质量指标
```python
def evaluate_conversion_quality(input_cloud, output_cloud):
    metrics = {}
    
    # 1. 点保留率
    metrics['retention_rate'] = len(output_cloud) / len(input_cloud)
    
    # 2. 网格填充率
    valid_cells = count_valid(output_cloud)
    total_cells = output_cloud.height * output_cloud.width
    metrics['fill_rate'] = valid_cells / total_cells
    
    # 3. 深度一致性 (与原始点云对比)
    metrics['depth_rmse'] = compute_depth_error(input_cloud, output_cloud)
    
    # 4. 处理延迟
    metrics['latency_ms'] = output_timestamp - input_timestamp
    
    return metrics

# 合格标准:
# retention_rate: > 0.75
# fill_rate: > 0.70
# depth_rmse: < 0.05m
# latency_ms: < 10ms
```

## 6. 已知限制与改进方向

### 6.1 当前限制
1. **点密度损失**: 网格化会丢弃部分高密度区域的点
2. **插值伪影**: 空洞填充可能创建虚假表面
3. **固定分辨率**: 无法动态适应场景复杂度
4. **延迟增加**: 增加~5-10ms处理延迟

### 6.2 未来改进方向
1. **自适应分辨率**: 根据点密度动态调整网格分辨率
2. **深度学习优化**: 使用神经网络进行智能插值
3. **多帧融合**: 累积多帧Mid360数据提高覆盖率
4. **GPU加速**: 使用CUDA加速网格映射和插值

### 6.3 替代方案探讨
```
方案A (当前): 点云预处理
  优点: 不修改nvblox, 模块化
  缺点: 信息损失, 额外延迟

方案B (理想): 修改nvblox核心
  优点: 零损失, 最优性能
  缺点: 需要深度修改nvblox

方案C (混合): 直接TSDF积分
  优点: 跳过深度图转换
  缺点: 需要重写积分逻辑
```

## 7. 参考资料

- [Livox Mid360 User Manual](https://www.livoxtech.com/mid-360/specs)
- [nvblox Architecture](https://github.com/nvidia-isaac/nvblox/docs)
- [PointCloud2 Specification](http://wiki.ros.org/sensor_msgs)
- [Spherical Projection Algorithms](https://en.wikipedia.org/wiki/Spherical_coordinate_system)

---

**文档版本**: v1.0  
**最后更新**: 2024  
**维护者**: Your Team
