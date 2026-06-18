cmake_minimum_required(VERSION 3.8)
project(mid360_to_nvblox_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)

# Check for CUDA
find_package(CUDA)
if(CUDA_FOUND)
  enable_language(CUDA)
  set(USE_CUDA ON)
  add_definitions(-DUSE_CUDA)
  message(STATUS "CUDA found - GPU acceleration enabled")
  message(STATUS "  CUDA version: ${CUDA_VERSION}")
  message(STATUS "  CUDA toolkit: ${CUDA_TOOLKIT_ROOT_DIR}")
else()
  set(USE_CUDA OFF)
  message(STATUS "CUDA not found - building CPU-only version")
endif()

# Include directories
include_directories(include)

# ============================================================================
# CPU-only version (always built)
# ============================================================================
add_library(mid360_bridge_node SHARED
  src/mid360_bridge_node.cpp
)

ament_target_dependencies(mid360_bridge_node
  rclcpp
  rclcpp_components
  sensor_msgs
)

rclcpp_components_register_nodes(mid360_bridge_node "mid360_bridge::Mid360BridgeNode")

# CPU-only executable
add_executable(mid360_bridge_node_exe
  src/mid360_bridge_node_main.cpp
)

target_link_libraries(mid360_bridge_node_exe
  mid360_bridge_node
)

ament_target_dependencies(mid360_bridge_node_exe
  rclcpp
)

# ============================================================================
# GPU-accelerated version (built if CUDA available)
# ============================================================================
if(USE_CUDA)
  # CUDA kernels library
  add_library(mid360_bridge_cuda SHARED
    src/cuda/bridge_converter_gpu.cu
  )
  
  set_target_properties(mid360_bridge_cuda PROPERTIES
    CUDA_SEPARABLE_COMPILATION ON
    CUDA_RESOLVE_DEVICE_SYMBOLS ON
  )
  
  # Set CUDA architectures
  # Jetson Orin: sm_87, Xavier: sm_72, AGX Xavier: sm_72
  set_property(TARGET mid360_bridge_cuda PROPERTY CUDA_ARCHITECTURES 72 87)
  
  target_compile_options(mid360_bridge_cuda PRIVATE
    $<$<COMPILE_LANGUAGE:CUDA>:
      --expt-relaxed-constexpr
      --use_fast_math
      --default-stream per-thread
      -lineinfo
    >
  )
  
  # GPU-enabled bridge node
  add_library(mid360_bridge_node_gpu SHARED
    src/mid360_bridge_node_gpu.cpp
  )
  
  target_link_libraries(mid360_bridge_node_gpu
    mid360_bridge_node
    mid360_bridge_cuda
  )
  
  ament_target_dependencies(mid360_bridge_node_gpu
    rclcpp
    rclcpp_components
    sensor_msgs
  )
  
  rclcpp_components_register_nodes(mid360_bridge_node_gpu "mid360_bridge::Mid360BridgeNodeGPU")
  
  # GPU-enabled executable
  add_executable(mid360_bridge_node_gpu_exe
    src/mid360_bridge_node_main.cpp
  )
  
  target_link_libraries(mid360_bridge_node_gpu_exe
    mid360_bridge_node_gpu
  )
  
  ament_target_dependencies(mid360_bridge_node_gpu_exe
    rclcpp
  )
endif()

# ============================================================================
# Install
# ============================================================================
install(TARGETS mid360_bridge_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS mid360_bridge_node_exe
  DESTINATION lib/${PROJECT_NAME}
)

if(USE_CUDA)
  install(TARGETS 
    mid360_bridge_cuda
    mid360_bridge_node_gpu
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib
  )

  install(TARGETS mid360_bridge_node_gpu_exe
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config scripts
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY docs
  DESTINATION share/${PROJECT_NAME}
)

# Make scripts executable
install(PROGRAMS
  scripts/bridge_diagnostics.py
  scripts/benchmark_cpu_vs_gpu.py
  DESTINATION lib/${PROJECT_NAME}
)

# ============================================================================
# Testing
# ============================================================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# ============================================================================
# Export
# ============================================================================
ament_export_include_directories(include)
ament_export_libraries(mid360_bridge_node)
if(USE_CUDA)
  ament_export_libraries(mid360_bridge_cuda mid360_bridge_node_gpu)
endif()
ament_export_dependencies(rclcpp rclcpp_components sensor_msgs)

ament_package()
