cmake_minimum_required(VERSION 3.8)
project(height_scan_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# UDP height scan sender
add_executable(udp_height_scan_sender src/udp_height_scan_sender.cpp)
target_include_directories(udp_height_scan_sender PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(udp_height_scan_sender PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(udp_height_scan_sender
  rclcpp
  std_msgs
)

# UDP safety guardian sender
add_executable(udp_safety_guardian_sender src/udp_safety_guardian_sender.cpp)
target_include_directories(udp_safety_guardian_sender PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(udp_safety_guardian_sender PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(udp_safety_guardian_sender
  rclcpp
  std_msgs
)

install(TARGETS udp_height_scan_sender udp_safety_guardian_sender
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
