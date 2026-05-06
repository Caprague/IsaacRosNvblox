cmake_minimum_required(VERSION 3.8)
project(safety_guardian)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 寻找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)

# 添加可执行文件
add_executable(safety_guardian_node
  src/safety_guardian_node.cpp
)

# 设置包含目录
target_include_directories(safety_guardian_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 链接依赖
ament_target_dependencies(safety_guardian_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  diagnostic_msgs
  tf2_ros
  tf2
)

# 安装目标
install(TARGETS
  safety_guardian_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装launch目录
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# 安装config目录
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# 安装脚本目录
install(DIRECTORY
  scripts
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()