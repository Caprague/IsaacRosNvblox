# Mid360 to nvblox Bridge

ROS2 package for converting Livox Mid360 unstructured pointcloud to nvblox-compatible structured format using CUDA GPU acceleration.

## Overview

Livox Mid360 uses a non-repetitive scanning pattern that produces unstructured pointclouds, which are incompatible with nvblox's expectation of structured lidar data. This bridge node converts Mid360 pointclouds into a virtual structured lidar format that nvblox can process.

### Key Features

- Converts unstructured Mid360 pointcloud to structured grid format
- GPU-accelerated via CUDA (requires Jetson Orin/Xavier or CUDA-capable GPU)
- Configurable virtual lidar model (resolution, FOV, range)
- Hole filling via neighbor interpolation
- Multiple aggregation strategies (mean/min/max)
- Full 360° horizontal coverage
- Performance monitoring and diagnostics

## How It Works

```
Mid360 Pointcloud          Bridge Node              nvblox
─────────────────    ───────────────────    ──────────────────
Unstructured    →    1. Spherical          →  Structured
width=N              2. Grid Mapping           width=1800
height=1             3. Interpolation          height=32
Random points        4. Reconstruction         Fixed grid
```

### Conversion Process

1. **Cartesian to Spherical**: Convert each 3D point (x,y,z) to (range, azimuth, elevation)
2. **Grid Mapping**: Map spherical coordinates to discrete grid cells
3. **Aggregation**: Handle multiple points in same cell (mean/min/max)
4. **Hole Filling**: Interpolate missing cells from neighbors
5. **Reconstruction**: Convert grid back to structured pointcloud

## Package Structure

```
mid360_to_nvblox_bridge/
├── include/mid360_to_nvblox_bridge/
│   ├── mid360_bridge_node_gpu.hpp       # GPU bridge node
│   └── cuda/
│       ├── bridge_converter_gpu.hpp     # Host-side GPU declarations
│       └── bridge_kernels.cuh           # CUDA kernels
├── src/
│   ├── mid360_bridge_node_gpu.cpp       # GPU node implementation
│   ├── mid360_bridge_node_gpu_main.cpp  # Executable entry point
│   └── cuda/
│       └── bridge_converter_gpu.cu      # CUDA implementation
├── config/
│   └── mid360_bridge_gpu.yaml           # GPU config
├── launch/
│   └── mid360_bridge.launch.py          # Launch file
├── scripts/
│   └── bridge_diagnostics.py            # Performance monitoring
├── docs/
│   ├── TECHNICAL_DESIGN.md
│   ├── DEPLOYMENT_GUIDE.md
│   ├── QUICK_REFERENCE.md
│   ├── GPU_ACCELERATION_ANALYSIS.md
│   ├── GPU_IMPLEMENTATION_SUMMARY.md
│   └── ASYNC_CUDA_COMPLETE_REPORT.md
├── CMakeLists.txt
├── package.xml
└── README.md
```

## Installation

**Requirements:**
- CUDA Toolkit (Jetson: pre-installed)
- Jetson Orin (sm_87) / Xavier (sm_72) / AGX Xavier (sm_72)

```bash
colcon build --packages-select mid360_to_nvblox_bridge \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

source install/setup.bash
```

## Configuration

Edit `config/mid360_bridge_gpu.yaml`:

```yaml
virtual_lidar_width: 1800      # Horizontal resolution (0.2° per pixel)
virtual_lidar_height: 32       # Vertical beams
min_range_m: 0.5               # Minimum valid range
max_range_m: 30.0              # Maximum valid range
min_elevation_deg: -7.0        # Match Mid360: -7°
max_elevation_deg: 52.0        # Match Mid360: +52°

aggregation_method: "mean"     # mean/min/max
enable_hole_filling: true
max_hole_fill_iterations: 2
```

### Matching nvblox Configuration

**CRITICAL**: nvblox config must match the virtual lidar parameters!

Edit `nvblox_base.yaml`:

```yaml
lidar_width: 1800
lidar_height: 32
lidar_min_valid_range_m: 0.5
lidar_max_valid_range_m: 30.0
use_non_equal_vertical_fov_lidar_params: true
min_angle_below_zero_elevation_rad: 0.122  # 7° in radians
max_angle_above_zero_elevation_rad: 0.908  # 52° in radians
```

## Usage

```bash
# Terminal 1: Start Mid360 driver
ros2 launch livox_ros_driver2 msg_MID360_launch.py

# Terminal 2: Start bridge node (GPU)
ros2 launch mid360_to_nvblox_bridge mid360_bridge.launch.py \
    input_pointcloud_topic:=/livox/lidar \
    output_pointcloud_topic:=/lidar/pointcloud_structured

# Terminal 3: Monitor performance (optional)
ros2 run mid360_to_nvblox_bridge bridge_diagnostics.py
```

### Verification

```bash
ros2 topic info /lidar/pointcloud_structured

# Expected:
# height: 32, width: 1800, is_dense: false
```

### Debug Mode

```bash
ros2 run mid360_to_nvblox_bridge mid360_bridge_node_exe \
    --ros-args --log-level debug
```

## Troubleshooting

### nvblox Rejects Pointcloud

**Symptom**: "LiDAR intrinsics are inconsistent"

**Solution**: Ensure parameter matching:
```bash
ros2 param get /mid360_bridge virtual_lidar_width
ros2 param get /mid360_bridge virtual_lidar_height
# Must match nvblox config exactly
```

### Many NaN Points in Output

- Check `min_elevation_deg` / `max_elevation_deg` match Mid360 FOV
- Increase `max_range_m` if too restrictive
- Enable `hole_filling` for sparse input data

### GPU Initialization Failure

- Verify CUDA device: `nvidia-smi`
- Check CUDA toolkit installation
- Ensure compatible GPU architecture (sm_72/sm_87)

## License

Apache-2.0
