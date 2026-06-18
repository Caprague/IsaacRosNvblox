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

# CUDA is required
find_package(CUDA REQUIRED)
enable_language(CUDA)
set(USE_CUDA ON)
add_definitions(-DUSE_CUDA)
message(STATUS "CUDA found - GPU acceleration enabled")
message(STATUS "  CUDA version: ${CUDA_VERSION}")
message(STATUS "  CUDA toolkit: ${CUDA_TOOLKIT_ROOT_DIR}")

# Include directories
include_directories(include)

# ============================================================================
# CUDA kernels library
# ============================================================================
add_library(mid360_bridge_cuda SHARED
  src/cuda/bridge_converter_gpu.cu
)

set_target_properties(mid360_bridge_cuda PROPERTIES
  CUDA_SEPARABLE_COMPILATION ON
  CUDA_RESOLVE_DEVICE_SYMBOLS ON
)

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

# ============================================================================
# Bridge node library (GPU only)
# ============================================================================
add_library(mid360_bridge_node SHARED
  src/mid360_bridge_node_gpu.cpp
)

target_link_libraries(mid360_bridge_node
  mid360_bridge_cuda
)

ament_target_dependencies(mid360_bridge_node
  rclcpp
  rclcpp_components
  sensor_msgs
)

rclcpp_components_register_nodes(mid360_bridge_node "mid360_bridge::Mid360BridgeNode")

# ============================================================================
# Standalone executable (GPU only)
# ============================================================================
add_executable(mid360_bridge_node_exe
  src/mid360_bridge_node_gpu_main.cpp
)

target_link_libraries(mid360_bridge_node_exe
  mid360_bridge_node
)

ament_target_dependencies(mid360_bridge_node_exe
  rclcpp
)

# ============================================================================
# Install
# ============================================================================
install(TARGETS mid360_bridge_cuda mid360_bridge_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

install(TARGETS mid360_bridge_node_exe
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config scripts
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY docs
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/bridge_diagnostics.py
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
ament_export_libraries(mid360_bridge_cuda mid360_bridge_node)
ament_export_dependencies(rclcpp rclcpp_components sensor_msgs)

ament_package()
