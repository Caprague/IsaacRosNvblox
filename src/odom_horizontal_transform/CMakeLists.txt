cmake_minimum_required(VERSION 3.8)
project(odom_horizontal_transform)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 寻找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)

# 添加可执行文件
add_executable(imu_horizontal_align
  src/imu_horizontal_align.cpp
)

# 设置包含目录
target_include_directories(imu_horizontal_align PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

# 链接依赖
target_link_libraries(imu_horizontal_align
  ${EIGEN3_LIBRARIES}
)

# 设置依赖项
ament_target_dependencies(imu_horizontal_align
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2_eigen
)

# 安装目标
install(TARGETS
imu_horizontal_align
  DESTINATION lib/${PROJECT_NAME}
)

# 安装launch目录
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
