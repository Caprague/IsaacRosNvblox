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

#ifndef SAFETY_GUARDIAN__SAFETY_GUARDIAN_NODE_HPP_
#define SAFETY_GUARDIAN__SAFETY_GUARDIAN_NODE_HPP_

#include <memory>
#include <mutex>
#include <vector>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>

namespace safety_guardian
{

/**
 * @brief 安全保护节点
 *
 * 该节点负责监控 VSLAM 定位和高程采样的安全性，并发布安全状态。
 * 实现锁死机制：一旦检测到不安全状态，状态将永久保持为 false。
 */
class SafetyGuardianNode : public rclcpp::Node
{
public:
  /**
   * @brief 构造函数
   */
  SafetyGuardianNode();

  /**
   * @brief 析构函数
   */
  ~SafetyGuardianNode();

private:
  // ========== 参数 ==========

  // VSLAM监控参数
  double vslam_timeout_threshold_;      // VSLAM超时阈值（秒）
  double vslam_position_threshold_;     // 位姿变化阈值（米）
  bool vslam_use_tf_;                   // 是否使用TF获取位姿
  std::string vslam_pose_topic_;        // 备选位姿话题

  // 高程频率监测参数
  double elevation_calibration_duration_;    // 采样校准时长（秒），用于自动计算期望频率
  double elevation_frequency_tolerance_;    // 频率容差比例（0~1），默认0.2

  // 安全状态发布参数
  double status_publish_rate_;          // 安全状态发布频率（Hz）

  // 诊断信息发布参数
  bool enable_diagnostics_;             // 是否发布诊断信息
  double diagnostics_rate_;             // 诊断信息发布频率（Hz）

  // TF参数
  std::string global_frame_;            // 全局坐标系（通常为 "map" 或 "odom"）
  std::string base_frame_;              // 机器人基座坐标系（通常为 "base_link"）

  // ========== 订阅器和发布器 ==========

  // 安全状态发布器
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr safety_status_pub_;

  // 诊断信息发布器
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;

  // 高程扫描订阅器
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr elevation_sub_;

  // TF相关
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ========== 定时器 ==========

  // 安全状态发布定时器
  rclcpp::TimerBase::SharedPtr status_timer_;

  // 诊断信息发布定时器
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;

  // VSLAM监控定时器
  rclcpp::TimerBase::SharedPtr vslam_monitor_timer_;

  // 信息汇总打印定时器
  rclcpp::TimerBase::SharedPtr summary_timer_;

  // ========== 安全状态 ==========

  // 线程安全保护
  mutable std::mutex safety_mutex_;

  // 安全状态
  bool safety_status_;                  // 当前安全状态
  bool safety_status_locked_;           // 是否已锁死

  // ========== VSLAM监控状态 ==========

  mutable std::mutex vslam_mutex_;

  rclcpp::Time last_vslam_update_time_; // 最后一次VSLAM更新时间
  geometry_msgs::msg::Point last_position_;  // 上一次位姿
  bool vslam_initialized_;              // VSLAM是否已初始化
  bool vslam_safe_;                     // VSLAM安全状态
  double last_position_change_;         // 最后一次位姿变化量（米）

  // ========== 高程频率监测状态 ==========

  mutable std::mutex elevation_mutex_;

  rclcpp::Time last_elevation_time_;            // 最后一次收到高程消息的时间
  rclcpp::Time elevation_calibration_start_;    // 校准采样开始时间
  double elevation_baselined_freq_;             // 校准得到的基准频率（Hz）
  double elevation_measured_frequency_;          // 实测高程消息频率（Hz）
  size_t elevation_calibration_count_;           // 校准期消息计数
  bool elevation_calibrated_;                    // 是否完成校准
  size_t elevation_received_count_;              // 高程消息接收计数
  std::vector<rclcpp::Time> elevation_timestamps_; // 滑动窗口时间戳

  // ========== 统计信息 ==========

  mutable std::mutex stats_mutex_;

  size_t total_vslam_updates_;              // VSLAM更新总次数
  size_t total_elevation_updates_;          // 高程消息接收总次数
  size_t vslam_timeout_count_;              // VSLAM超时次数
  size_t vslam_position_jump_count_;        // VSLAM位姿跳跃次数
  rclcpp::Time node_start_time_;            // 节点启动时间

  // ========== 回调函数 ==========

  /**
   * @brief 安全状态发布定时器回调
   */
  void statusTimerCallback();

  /**
   * @brief 诊断信息发布定时器回调
   */
  void diagnosticsTimerCallback();

  /**
   * @brief VSLAM监控定时器回调
   */
  void vslamMonitorTimerCallback();

  /**
   * @brief 信息汇总打印定时器回调
   */
  void summaryTimerCallback();

  /**
   * @brief 高程消息回调（频率监测）
   * @param msg 高程扫描消息
   */
  void elevationCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

  // ========== 辅助函数 ==========

  /**
   * @brief 更新安全状态（仅由VSLAM驱动）
   * @param vslam_safe VSLAM是否安全
   */
  void updateSafetyStatus(bool vslam_safe);

  /**
   * @brief 获取当前安全状态
   * @return 安全状态
   */
  bool getSafetyStatus() const;

  /**
   * @brief 计算两点之间的欧氏距离
   * @param p1 第一个点
   * @param p2 第二个点
   * @return 欧氏距离
   */
  double calculateDistance(
    const geometry_msgs::msg::Point & p1,
    const geometry_msgs::msg::Point & p2) const;

  /**
   * @brief 计算高程消息实测频率
   * @return 实测频率（Hz），数据不足返回0
   */
  double calculateElevationFrequency() const;

  /**
   * @brief 获取高程监测状态描述（用于打印）
   * @param now 当前时间
   * @return true=绿色(正常), false=黄色(警告)
   */
  bool getElevationMonitorOk(const rclcpp::Time & now) const;

  /**
   * @brief 发布诊断信息
   */
  void publishDiagnostics();

  /**
   * @brief 从TF获取当前位姿
   * @param position 输出位姿
   * @return 是否成功获取
   */
  bool getCurrentPositionFromTF(geometry_msgs::msg::Point & position);

  /**
   * @brief 监控VSLAM状态
   */
  void monitorVSLAMStatus();
};

}  // namespace safety_guardian

#endif  // SAFETY_GUARDIAN__SAFETY_GUARDIAN_NODE_HPP_