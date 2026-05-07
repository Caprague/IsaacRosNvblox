// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// SPDX-License-Identifier: Apache-2.0

#include "safety_guardian/safety_guardian_node.hpp"

#include <cmath>
#include <memory>
#include <iostream>
#include <iomanip>

namespace safety_guardian
{

SafetyGuardianNode::SafetyGuardianNode()
: Node("safety_guardian_node")
{
  RCLCPP_INFO(this->get_logger(), "Safety Guardian Node is starting...");

  // ========== 声明参数 ==========

  // VSLAM监控参数
  this->declare_parameter<double>("vslam_timeout_threshold", 0.5);
  this->declare_parameter<double>("vslam_position_threshold", 1.0);
  this->declare_parameter<bool>("vslam_use_tf", true);
  this->declare_parameter<std::string>("vslam_pose_topic", "/visual_slam/tracking/vo_pose");

  // 高程频率监测参数
  this->declare_parameter<double>("elevation_calibration_duration", 3.0);
  this->declare_parameter<double>("elevation_frequency_tolerance", 0.2);

  // 安全状态发布参数
  this->declare_parameter<double>("status_publish_rate", 10.0);

  // 诊断信息发布参数
  this->declare_parameter<bool>("enable_diagnostics", true);
  this->declare_parameter<double>("diagnostics_rate", 1.0);

  // TF参数
  this->declare_parameter<std::string>("global_frame", "odom");
  this->declare_parameter<std::string>("base_frame", "base_link");

  // ========== 获取参数 ==========

  this->vslam_timeout_threshold_ = this->get_parameter("vslam_timeout_threshold").as_double();
  this->vslam_position_threshold_ = this->get_parameter("vslam_position_threshold").as_double();
  this->vslam_use_tf_ = this->get_parameter("vslam_use_tf").as_bool();
  this->vslam_pose_topic_ = this->get_parameter("vslam_pose_topic").as_string();

  this->elevation_calibration_duration_ = this->get_parameter("elevation_calibration_duration").as_double();
  this->elevation_frequency_tolerance_ = this->get_parameter("elevation_frequency_tolerance").as_double();

  this->status_publish_rate_ = this->get_parameter("status_publish_rate").as_double();

  this->enable_diagnostics_ = this->get_parameter("enable_diagnostics").as_bool();
  this->diagnostics_rate_ = this->get_parameter("diagnostics_rate").as_double();

  this->global_frame_ = this->get_parameter("global_frame").as_string();
  this->base_frame_ = this->get_parameter("base_frame").as_string();

  RCLCPP_INFO(this->get_logger(), "Parameters loaded:");
  RCLCPP_INFO(this->get_logger(), "  VSLAM timeout threshold: %.2f s", this->vslam_timeout_threshold_);
  RCLCPP_INFO(this->get_logger(), "  VSLAM position threshold: %.2f m", this->vslam_position_threshold_);
  RCLCPP_INFO(this->get_logger(), "  Elevation calibration duration: %.1f s", this->elevation_calibration_duration_);
  RCLCPP_INFO(this->get_logger(), "  Elevation frequency tolerance: %.2f", this->elevation_frequency_tolerance_);
  RCLCPP_INFO(this->get_logger(), "  Status publish rate: %.2f Hz", this->status_publish_rate_);
  RCLCPP_INFO(this->get_logger(), "  Global frame: %s", this->global_frame_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Base frame: %s", this->base_frame_.c_str());

  // ========== 初始化安全状态 ==========

  this->safety_status_ = true;
  this->safety_status_locked_ = false;

  // ========== 初始化VSLAM监控状态 ==========

  this->last_vslam_update_time_ = this->get_clock()->now();
  this->vslam_initialized_ = false;
  this->vslam_safe_ = true;
  this->last_position_change_ = 0.0;

  // ========== 初始化高程频率监测状态 ==========

  this->elevation_measured_frequency_ = 0.0;
  this->elevation_baselined_freq_ = 0.0;
  this->elevation_calibration_count_ = 0;
  this->elevation_received_count_ = 0;
  this->elevation_calibrated_ = false;
  this->last_elevation_time_ = this->get_clock()->now();
  this->elevation_calibration_start_ = this->get_clock()->now();

  // ========== 初始化统计信息 ==========

  this->total_vslam_updates_ = 0;
  this->total_elevation_updates_ = 0;
  this->vslam_timeout_count_ = 0;
  this->vslam_position_jump_count_ = 0;
  this->node_start_time_ = this->get_clock()->now();

  // ========== 创建发布器 ==========
  auto qos_pub = rclcpp::SystemDefaultsQoS();
  qos_pub.keep_last(10); 

  this->safety_status_pub_ = this->create_publisher<std_msgs::msg::Bool>(
    "/safety_guardian/status", qos_pub);

  if (this->enable_diagnostics_) {
    this->diagnostics_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/safety_guardian/diagnostics", qos_pub);
  }

  // ========== 创建订阅器 ==========
  auto qos_sub = rclcpp::SystemDefaultsQoS();

  this->elevation_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
    "/nvblox_node/locomotion_height_scan",
    qos_sub,
    std::bind(&SafetyGuardianNode::elevationCallback, this, std::placeholders::_1)
  );

  RCLCPP_INFO(this->get_logger(), "Subscribed to elevation scan topic: /nvblox_node/locomotion_height_scan");

  // ========== 初始化TF ==========

  if (this->vslam_use_tf_) {
    this->tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    this->tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*this->tf_buffer_);
    RCLCPP_INFO(this->get_logger(), "TF listener initialized for %s -> %s",
                this->global_frame_.c_str(), this->base_frame_.c_str());
  }

  // ========== 创建定时器 ==========

  double status_period = 1.0 / this->status_publish_rate_;
  this->status_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(status_period),
    std::bind(&SafetyGuardianNode::statusTimerCallback, this)
  );

  if (this->enable_diagnostics_) {
    double diagnostics_period = 1.0 / this->diagnostics_rate_;
    this->diagnostics_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(diagnostics_period),
      std::bind(&SafetyGuardianNode::diagnosticsTimerCallback, this)
    );
  }

  this->vslam_monitor_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(20),
    std::bind(&SafetyGuardianNode::vslamMonitorTimerCallback, this)
  );

  this->summary_timer_ = this->create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&SafetyGuardianNode::summaryTimerCallback, this)
  );

  RCLCPP_INFO(this->get_logger(), "Safety Guardian Node initialized successfully");
}

SafetyGuardianNode::~SafetyGuardianNode()
{
  RCLCPP_INFO(this->get_logger(), "Safety Guardian Node is shutting down...");
}

void SafetyGuardianNode::statusTimerCallback()
{
  std_msgs::msg::Bool msg;
  msg.data = this->getSafetyStatus();
  this->safety_status_pub_->publish(msg);
}

void SafetyGuardianNode::diagnosticsTimerCallback()
{
  this->publishDiagnostics();
}

void SafetyGuardianNode::vslamMonitorTimerCallback()
{
  this->monitorVSLAMStatus();
}

void SafetyGuardianNode::summaryTimerCallback()
{
  std::lock_guard<std::mutex> lock_safety(this->safety_mutex_);

  if (this->safety_status_locked_) {
    return;
  }

  // ANSI颜色代码
  const std::string RESET = "\033[0m";
  const std::string RED = "\033[31m";
  const std::string GREEN = "\033[32m";
  const std::string YELLOW = "\033[33m";
  const std::string BLUE = "\033[34m";
  const std::string CYAN = "\033[36m";
  const std::string BOLD = "\033[1m";

  rclcpp::Time now = this->get_clock()->now();
  double uptime = (now - this->node_start_time_).seconds();

  std::cout << "\n" << CYAN << BOLD << "========================================" << RESET << std::endl;
  std::cout << CYAN << BOLD << "Safety Guardian - Status Summary" << RESET << std::endl;
  std::cout << CYAN << BOLD << "========================================" << RESET << std::endl;
  std::cout << "Uptime: " << std::fixed << std::setprecision(1) << uptime << " s" << std::endl;
  std::cout << BLUE << "----------------------------------------" << RESET << std::endl;

  // --- 安全状态（仅 VSLAM 驱动） ---
  std::cout << BOLD << "Safety Status (VSLAM-driven):" << RESET << std::endl;
  std::cout << "  Overall:        "
            << (this->safety_status_ ? GREEN : RED)
            << (this->safety_status_ ? "SAFE" : "UNSAFE")
            << RESET << std::endl;
  std::cout << "  Locked:         "
            << (this->safety_status_locked_ ? RED : GREEN)
            << (this->safety_status_locked_ ? "YES" : "NO")
            << RESET << std::endl;
  std::cout << BLUE << "----------------------------------------" << RESET << std::endl;

  // --- VSLAM 监控 ---
  std::cout << BOLD << "VSLAM Monitor:" << RESET << std::endl;
  {
    std::lock_guard<std::mutex> lock(this->vslam_mutex_);
    std::cout << "  Status:         "
              << (this->vslam_safe_ ? GREEN : RED)
              << (this->vslam_safe_ ? "SAFE" : "UNSAFE")
              << RESET << std::endl;
    std::cout << "  Initialized:    "
              << (this->vslam_initialized_ ? GREEN : YELLOW)
              << (this->vslam_initialized_ ? "YES" : "NO")
              << RESET << std::endl;
    std::cout << "  Total Updates:  " << this->total_vslam_updates_ << std::endl;
    std::cout << "  Timeout Count:  "
              << (this->vslam_timeout_count_ > 0 ? RED : GREEN)
              << this->vslam_timeout_count_
              << RESET << std::endl;
    std::cout << "  Position Jumps: "
              << (this->vslam_position_jump_count_ > 0 ? RED : GREEN)
              << this->vslam_position_jump_count_
              << RESET << std::endl;
    std::cout << "  Last Change:    " << std::fixed << std::setprecision(3)
              << this->last_position_change_ << " m" << std::endl;
    std::cout << "  Current Pos:    [" << std::fixed << std::setprecision(2)
              << this->last_position_.x << ", "
              << this->last_position_.y << ", "
              << this->last_position_.z << "]" << std::endl;
  }
  std::cout << BLUE << "----------------------------------------" << RESET << std::endl;

  // --- 高程频率监测（独立于安全状态） ---
  std::cout << BOLD << "Elevation Freq Monitor (advisory):" << RESET << std::endl;
  {
    std::lock_guard<std::mutex> lock(this->elevation_mutex_);
    bool elevation_ok = this->getElevationMonitorOk(now);

    if (!this->elevation_calibrated_) {
      double elapsed = (now - this->elevation_calibration_start_).seconds();
      double remaining = this->elevation_calibration_duration_ - elapsed;
      std::cout << "  Status:         " << YELLOW << "CALIBRATING" << RESET
                << " (" << std::fixed << std::setprecision(1)
                << std::max(remaining, 0.0) << " s remaining)" << std::endl;
      std::cout << "  Samples:        " << this->elevation_calibration_count_ << std::endl;
    } else if (elevation_ok) {
      std::cout << "  Status:         " << GREEN << "OK" << RESET << std::endl;
    } else {
      std::cout << "  Status:         " << YELLOW << "WARN" << RESET << std::endl;
    }

    if (this->elevation_calibrated_) {
      double timeout_threshold = 1.0 / this->elevation_baselined_freq_;
      double time_since = (now - this->last_elevation_time_).seconds();
      std::cout << "  Baseline Freq:  " << std::fixed << std::setprecision(1)
                << this->elevation_baselined_freq_ << " Hz" << std::endl;
      std::cout << "  Measured Freq:  " << std::fixed << std::setprecision(1)
                << this->elevation_measured_frequency_ << " Hz" << std::endl;
      std::cout << "  Timeout Limit:  " << std::fixed << std::setprecision(3)
                << timeout_threshold << " s (1 period)" << std::endl;
      std::cout << "  Time Since Msg: " << std::fixed << std::setprecision(3)
                << time_since << " s" << std::endl;
    }
    std::cout << "  Msgs Received:  " << this->elevation_received_count_ << std::endl;
  }
  std::cout << CYAN << BOLD << "========================================" << RESET << std::endl;
  std::cout << std::flush;
}

void SafetyGuardianNode::elevationCallback(const std_msgs::msg::Float32MultiArray::SharedPtr /*msg*/)
{
  std::lock_guard<std::mutex> lock(this->elevation_mutex_);

  rclcpp::Time now = this->get_clock()->now();
  this->elevation_received_count_++;
  this->total_elevation_updates_++;
  this->last_elevation_time_ = now;

  if (!this->elevation_calibrated_) {
    // ---- 校准阶段 ----
    if (this->elevation_calibration_count_ == 0) {
      // 第一条消息，记录校准起始时间
      this->elevation_calibration_start_ = now;
      RCLCPP_INFO(this->get_logger(), "Elevation calibration started (%.1f s sampling)...",
                  this->elevation_calibration_duration_);
    }

    this->elevation_calibration_count_++;

    double elapsed = (now - this->elevation_calibration_start_).seconds();
    if (elapsed >= this->elevation_calibration_duration_) {
      // 校准结束，计算基准频率
      this->elevation_baselined_freq_ =
        static_cast<double>(this->elevation_calibration_count_ - 1) / elapsed;
      this->elevation_calibrated_ = true;
      this->elevation_measured_frequency_ = this->elevation_baselined_freq_;
      RCLCPP_INFO(this->get_logger(),
                  "Elevation calibration done: baseline freq = %.1f Hz (from %zu samples in %.2f s)",
                  this->elevation_baselined_freq_, this->elevation_calibration_count_, elapsed);
    }
    return;
  }

  // ---- 监测阶段：滑动窗口计算实测频率 ----
  this->elevation_timestamps_.push_back(now);
  // 窗口大小：基准频率 * 2秒，至少10条
  size_t window_size = std::max(
    static_cast<size_t>(this->elevation_baselined_freq_ * 2.0), size_t(10));
  while (this->elevation_timestamps_.size() > window_size) {
    this->elevation_timestamps_.erase(this->elevation_timestamps_.begin());
  }

  this->elevation_measured_frequency_ = this->calculateElevationFrequency();
}

void SafetyGuardianNode::updateSafetyStatus(bool vslam_safe)
{
  std::lock_guard<std::mutex> lock(this->safety_mutex_);

  if (!this->safety_status_locked_) {
    bool previous_status = this->safety_status_;
    this->safety_status_ = vslam_safe;

    if (previous_status != this->safety_status_) {
      if (!this->safety_status_) {
        this->safety_status_locked_ = true;
        RCLCPP_ERROR(this->get_logger(), "========================================");
        RCLCPP_ERROR(this->get_logger(), "SAFETY VIOLATION DETECTED!");
        RCLCPP_ERROR(this->get_logger(), "========================================");
        RCLCPP_ERROR(this->get_logger(), "System is being LOCKED DOWN.");
        RCLCPP_ERROR(this->get_logger(), "Status summary:");
        RCLCPP_ERROR(this->get_logger(), "  VSLAM monitor:    %s", vslam_safe ? "SAFE" : "UNSAFE");
        RCLCPP_ERROR(this->get_logger(), "  Overall status:   UNSAFE");
        RCLCPP_ERROR(this->get_logger(), "Action required:");
        RCLCPP_ERROR(this->get_logger(), "  1. Stop all robot operations immediately");
        RCLCPP_ERROR(this->get_logger(), "  2. Investigate the root cause");
        RCLCPP_ERROR(this->get_logger(), "  3. Restart the system after fixing the issue");
        RCLCPP_ERROR(this->get_logger(), "========================================");
      }
    }
  } else {
    static auto last_lockdown_reminder = this->get_clock()->now();
    auto current_time = this->get_clock()->now();
    if ((current_time - last_lockdown_reminder).seconds() > 10.0) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
                           "SYSTEM REMAINS LOCKED DOWN - Manual intervention required");
      last_lockdown_reminder = current_time;
    }
  }
}

bool SafetyGuardianNode::getSafetyStatus() const
{
  std::lock_guard<std::mutex> lock(this->safety_mutex_);
  return this->safety_status_;
}

double SafetyGuardianNode::calculateDistance(
  const geometry_msgs::msg::Point & p1,
  const geometry_msgs::msg::Point & p2) const
{
  double dx = p1.x - p2.x;
  double dy = p1.y - p2.y;
  double dz = p1.z - p2.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double SafetyGuardianNode::calculateElevationFrequency() const
{
  if (this->elevation_timestamps_.size() < 2) {
    return 0.0;
  }
  double duration = (this->elevation_timestamps_.back() - this->elevation_timestamps_.front()).seconds();
  if (duration <= 0.0) {
    return 0.0;
  }
  return static_cast<double>(this->elevation_timestamps_.size() - 1) / duration;
}

bool SafetyGuardianNode::getElevationMonitorOk(const rclcpp::Time & now) const
{
  if (!this->elevation_calibrated_) {
    return false;
  }

  // 超时阈值 = 1个周期
  double timeout_threshold = 1.0 / this->elevation_baselined_freq_;
  double time_since_last = (now - this->last_elevation_time_).seconds();
  if (time_since_last > timeout_threshold) {
    return false;
  }

  // 频率检查：实测频率低于 基准*(1-容差) 则异常
  double min_freq = this->elevation_baselined_freq_ * (1.0 - this->elevation_frequency_tolerance_);
  if (this->elevation_measured_frequency_ < min_freq) {
    return false;
  }

  return true;
}

void SafetyGuardianNode::publishDiagnostics()
{
  diagnostic_msgs::msg::DiagnosticArray diag_array;
  diag_array.header.stamp = this->get_clock()->now();

  // VSLAM监控诊断
  diagnostic_msgs::msg::DiagnosticStatus vslam_status;
  vslam_status.name = "VSLAM Monitor";
  vslam_status.hardware_id = "VSLAM";

  {
    std::lock_guard<std::mutex> lock(this->vslam_mutex_);

    rclcpp::Time now = this->get_clock()->now();
    double time_since_update = (now - this->last_vslam_update_time_).seconds();

    diagnostic_msgs::msg::KeyValue kv1;
    kv1.key = "Time since last update";
    kv1.value = std::to_string(time_since_update) + " s";
    vslam_status.values.push_back(kv1);

    diagnostic_msgs::msg::KeyValue kv2;
    kv2.key = "Timeout threshold";
    kv2.value = std::to_string(this->vslam_timeout_threshold_) + " s";
    vslam_status.values.push_back(kv2);

    diagnostic_msgs::msg::KeyValue kv3;
    kv3.key = "VSLAM initialized";
    kv3.value = this->vslam_initialized_ ? "true" : "false";
    vslam_status.values.push_back(kv3);

    diagnostic_msgs::msg::KeyValue kv4;
    kv4.key = "Using TF";
    kv4.value = this->vslam_use_tf_ ? "true" : "false";
    vslam_status.values.push_back(kv4);

    diagnostic_msgs::msg::KeyValue kv5;
    kv5.key = "Global frame";
    kv5.value = this->global_frame_;
    vslam_status.values.push_back(kv5);

    diagnostic_msgs::msg::KeyValue kv6;
    kv6.key = "Base frame";
    kv6.value = this->base_frame_;
    vslam_status.values.push_back(kv6);

    if (this->vslam_initialized_) {
      diagnostic_msgs::msg::KeyValue kv7;
      kv7.key = "Current position";
      kv7.value = "[" + std::to_string(this->last_position_.x) + ", " +
                   std::to_string(this->last_position_.y) + ", " +
                   std::to_string(this->last_position_.z) + "]";
      vslam_status.values.push_back(kv7);

      diagnostic_msgs::msg::KeyValue kv8;
      kv8.key = "Position threshold";
      kv8.value = std::to_string(this->vslam_position_threshold_) + " m";
      vslam_status.values.push_back(kv8);
    }

    if (this->vslam_safe_) {
      vslam_status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      if (this->vslam_initialized_) {
        vslam_status.message = "VSLAM operating normally";
      } else {
        vslam_status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        vslam_status.message = "VSLAM initializing...";
      }
    } else {
      vslam_status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      if (time_since_update > this->vslam_timeout_threshold_) {
        vslam_status.message = "VSLAM timeout - no updates for " +
                               std::to_string(time_since_update) + " seconds";
      } else {
        vslam_status.message = "VSLAM position jump detected";
      }
    }
  }

  diag_array.status.push_back(vslam_status);

  // 高程频率监测诊断（advisory，不影响安全状态）
  diagnostic_msgs::msg::DiagnosticStatus elevation_status;
  elevation_status.name = "Elevation Freq Monitor";
  elevation_status.hardware_id = "Elevation";

  {
    std::lock_guard<std::mutex> lock(this->elevation_mutex_);
    rclcpp::Time now = this->get_clock()->now();

    diagnostic_msgs::msg::KeyValue kv1;
    kv1.key = "Calibrated";
    kv1.value = this->elevation_calibrated_ ? "true" : "false";
    elevation_status.values.push_back(kv1);

    if (this->elevation_calibrated_) {
      diagnostic_msgs::msg::KeyValue kv2;
      kv2.key = "Baseline frequency";
      kv2.value = std::to_string(this->elevation_baselined_freq_) + " Hz";
      elevation_status.values.push_back(kv2);

      diagnostic_msgs::msg::KeyValue kv3;
      kv3.key = "Measured frequency";
      kv3.value = std::to_string(this->elevation_measured_frequency_) + " Hz";
      elevation_status.values.push_back(kv3);

      diagnostic_msgs::msg::KeyValue kv4;
      kv4.key = "Timeout threshold (1 period)";
      kv4.value = std::to_string(1.0 / this->elevation_baselined_freq_) + " s";
      elevation_status.values.push_back(kv4);
    }

    diagnostic_msgs::msg::KeyValue kv5;
    kv5.key = "Messages received";
    kv5.value = std::to_string(this->elevation_received_count_);
    elevation_status.values.push_back(kv5);

    bool elevation_ok = this->getElevationMonitorOk(now);
    if (!this->elevation_calibrated_) {
      elevation_status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      elevation_status.message = "Elevation frequency calibrating...";
    } else if (elevation_ok) {
      elevation_status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      elevation_status.message = "Elevation frequency normal";
    } else {
      elevation_status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      double timeout_threshold = 1.0 / this->elevation_baselined_freq_;
      double time_since = (now - this->last_elevation_time_).seconds();
      if (time_since > timeout_threshold) {
        elevation_status.message = "Elevation message timeout: " +
                                   std::to_string(time_since) + " s since last msg (limit: " +
                                   std::to_string(timeout_threshold) + " s)";
      } else {
        elevation_status.message = "Elevation frequency below baseline: " +
                                   std::to_string(this->elevation_measured_frequency_) + " Hz (min: " +
                                   std::to_string(this->elevation_baselined_freq_ * (1.0 - this->elevation_frequency_tolerance_)) + " Hz)";
      }
    }
  }

  diag_array.status.push_back(elevation_status);

  // 整体安全状态诊断（仅由 VSLAM 驱动）
  diagnostic_msgs::msg::DiagnosticStatus overall_status;
  overall_status.name = "Overall Safety Status";
  overall_status.hardware_id = "Safety Guardian";

  {
    std::lock_guard<std::mutex> lock(this->safety_mutex_);

    diagnostic_msgs::msg::KeyValue kv1;
    kv1.key = "Safety status";
    kv1.value = this->safety_status_ ? "true" : "false";
    overall_status.values.push_back(kv1);

    diagnostic_msgs::msg::KeyValue kv2;
    kv2.key = "Locked";
    kv2.value = this->safety_status_locked_ ? "true" : "false";
    overall_status.values.push_back(kv2);

    diagnostic_msgs::msg::KeyValue kv3;
    kv3.key = "VSLAM safe";
    kv3.value = this->vslam_safe_ ? "true" : "false";
    overall_status.values.push_back(kv3);

    diagnostic_msgs::msg::KeyValue kv4;
    kv4.key = "Status publish rate";
    kv4.value = std::to_string(this->status_publish_rate_) + " Hz";
    overall_status.values.push_back(kv4);

    rclcpp::Time now = this->get_clock()->now();
    double uptime = (now - this->node_start_time_).seconds();

    diagnostic_msgs::msg::KeyValue kv5;
    kv5.key = "Uptime";
    kv5.value = std::to_string(uptime) + " s";
    overall_status.values.push_back(kv5);

    diagnostic_msgs::msg::KeyValue kv6;
    kv6.key = "Total VSLAM updates";
    kv6.value = std::to_string(this->total_vslam_updates_);
    overall_status.values.push_back(kv6);

    diagnostic_msgs::msg::KeyValue kv7;
    kv7.key = "VSLAM timeout count";
    kv7.value = std::to_string(this->vslam_timeout_count_);
    overall_status.values.push_back(kv7);

    diagnostic_msgs::msg::KeyValue kv8;
    kv8.key = "VSLAM position jump count";
    kv8.value = std::to_string(this->vslam_position_jump_count_);
    overall_status.values.push_back(kv8);

    diagnostic_msgs::msg::KeyValue kv9;
    kv9.key = "Total elevation updates";
    kv9.value = std::to_string(this->total_elevation_updates_);
    overall_status.values.push_back(kv9);

    if (this->safety_status_) {
      overall_status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      overall_status.message = "System is safe - VSLAM operating normally";
    } else {
      overall_status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      if (this->safety_status_locked_) {
        overall_status.message = "SYSTEM LOCKED DOWN - VSLAM safety violation detected";
      } else {
        overall_status.message = "System is unsafe";
      }
    }
  }

  diag_array.status.push_back(overall_status);

  this->diagnostics_pub_->publish(diag_array);
}

bool SafetyGuardianNode::getCurrentPositionFromTF(geometry_msgs::msg::Point & position)
{
  try {
    geometry_msgs::msg::TransformStamped transform;
    transform = this->tf_buffer_->lookupTransform(
      this->global_frame_,
      this->base_frame_,
      tf2::TimePointZero,
      tf2::durationFromSec(0.1));

    position.x = transform.transform.translation.x;
    position.y = transform.transform.translation.y;
    position.z = transform.transform.translation.z;

    return true;
  } catch (tf2::LookupException & ex) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "TF lookup failed: %s", ex.what());
    return false;
  } catch (tf2::ConnectivityException & ex) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "TF connectivity failed: %s", ex.what());
    return false;
  } catch (tf2::ExtrapolationException & ex) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "TF extrapolation failed: %s", ex.what());
    return false;
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "TF lookup failed: %s", ex.what());
    return false;
  }
}

void SafetyGuardianNode::monitorVSLAMStatus()
{
  std::lock_guard<std::mutex> lock(this->vslam_mutex_);

  if (!this->vslam_use_tf_) {
    this->vslam_safe_ = true;
    return;
  }

  geometry_msgs::msg::Point current_position;
  bool success = this->getCurrentPositionFromTF(current_position);

  if (!success) {
    rclcpp::Time now = this->get_clock()->now();
    double time_since_update = (now - this->last_vslam_update_time_).seconds();

    if (time_since_update > this->vslam_timeout_threshold_) {
      this->vslam_safe_ = false;
      this->vslam_timeout_count_++;
      RCLCPP_ERROR(this->get_logger(),
                   "VSLAM TIMEOUT: %.2f s since last update (threshold: %.2f s)",
                   time_since_update, this->vslam_timeout_threshold_);
      RCLCPP_ERROR(this->get_logger(),
                   "Possible causes: VSLAM node stopped, TF tree broken, or network issue");
    } else {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                          "TF lookup failed, but timeout not reached yet: %.2f / %.2f s",
                          time_since_update, this->vslam_timeout_threshold_);
    }
    this->updateSafetyStatus(this->vslam_safe_);
    return;
  }

  this->last_vslam_update_time_ = this->get_clock()->now();
  this->total_vslam_updates_++;

  if (this->vslam_initialized_) {
    double position_change = this->calculateDistance(current_position, this->last_position_);
    this->last_position_change_ = position_change;

    if (position_change > this->vslam_position_threshold_) {
      this->vslam_safe_ = false;
      this->vslam_position_jump_count_++;
      RCLCPP_ERROR(this->get_logger(),
                   "VSLAM POSITION JUMP DETECTED: %.2f m (threshold: %.2f m)",
                   position_change, this->vslam_position_threshold_);
      RCLCPP_ERROR(this->get_logger(),
                   "Previous position: [%.3f, %.3f, %.3f]",
                   this->last_position_.x, this->last_position_.y, this->last_position_.z);
      RCLCPP_ERROR(this->get_logger(),
                   "Current position:  [%.3f, %.3f, %.3f]",
                   current_position.x, current_position.y, current_position.z);
    } else {
      this->vslam_safe_ = true;
      static auto last_debug_time = this->get_clock()->now();
      auto current_time = this->get_clock()->now();
      if ((current_time - last_debug_time).seconds() > 5.0) {
        RCLCPP_DEBUG(this->get_logger(),
                     "VSLAM position change: %.3f m (OK)",
                     position_change);
        last_debug_time = current_time;
      }
    }
  } else {
    this->vslam_initialized_ = true;
    this->vslam_safe_ = true;
    this->last_position_ = current_position;
    this->last_position_change_ = 0.0;
    RCLCPP_INFO(this->get_logger(), "VSLAM initialized successfully");
    RCLCPP_INFO(this->get_logger(), "Initial position: [%.3f, %.3f, %.3f]",
                current_position.x, current_position.y, current_position.z);
    return;
  }

  this->last_position_ = current_position;

  // 安全状态仅由 VSLAM 驱动
  this->updateSafetyStatus(this->vslam_safe_);
}

}  // namespace safety_guardian

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<safety_guardian::SafetyGuardianNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
