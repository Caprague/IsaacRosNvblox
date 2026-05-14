#ifndef HEIGHT_SCAN_BRIDGE__UDP_HEIGHT_SCAN_SENDER_HPP_
#define HEIGHT_SCAN_BRIDGE__UDP_HEIGHT_SCAN_SENDER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <atomic>

class UDPHeightScanSender : public rclcpp::Node
{
public:
  UDPHeightScanSender();
  ~UDPHeightScanSender();

private:
  void callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  void print_stats();

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr stats_timer_;

  int sock_;
  struct sockaddr_in addr_;
  std::string target_ip_;
  int port_;
  bool print_stats_;

  std::atomic<int> send_count_{0};
  std::atomic<int> frame_count_since_last_{0};
  size_t last_bytes_per_frame_{0};
  size_t last_floats_per_frame_{0};
  double last_stats_time_{0.0};
};

#endif  // HEIGHT_SCAN_BRIDGE__UDP_HEIGHT_SCAN_SENDER_HPP_
