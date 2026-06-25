#include "height_scan_bridge/udp_safety_guardian_sender.hpp"
#include <cstring>

UDPSafetyGuardianSender::UDPSafetyGuardianSender()
: Node("safety_guardian_udp_sender"),
  sock_(-1),
  send_count_(0),
  frame_count_since_last_(0)
{
  this->declare_parameter<std::string>("target_ip", "192.168.123.18");
  this->declare_parameter<int>("port", 9871);
  this->declare_parameter<bool>("print_stats", false);

  target_ip_ = this->get_parameter("target_ip").as_string();
  port_ = this->get_parameter("port").as_int();
  print_stats_ = this->get_parameter("print_stats").as_bool();

  sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/safety_guardian/status", 10,
    [this](const std_msgs::msg::Bool::SharedPtr msg) { this->callback(msg); });

  // Create UDP socket
  sock_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (sock_ < 0) {
    RCLCPP_FATAL(this->get_logger(), "Failed to create UDP socket");
    return;
  }

  std::memset(&addr_, 0, sizeof(addr_));
  addr_.sin_family = AF_INET;
  addr_.sin_port = htons(port_);
  if (inet_pton(AF_INET, target_ip_.c_str(), &addr_.sin_addr) <= 0) {
    RCLCPP_FATAL(this->get_logger(), "Invalid target IP: %s", target_ip_.c_str());
    close(sock_);
    sock_ = -1;
    return;
  }

  // Send a test packet on startup to verify connectivity
  send_test_packet();

  RCLCPP_INFO(this->get_logger(),
    "Sending safety_guardian status via UDP to %s:%d (1 bool, 1 byte)",
    target_ip_.c_str(), port_);

  if (print_stats_) {
    stats_timer_ = this->create_wall_timer(
      std::chrono::seconds(3),
      [this]() { this->print_stats(); });
  }
}

UDPSafetyGuardianSender::~UDPSafetyGuardianSender()
{
  if (sock_ >= 0) {
    close(sock_);
  }
}

void UDPSafetyGuardianSender::send_test_packet()
{
  uint8_t test_data = 1;  // true = SAFE
  ssize_t n = sendto(sock_, &test_data, sizeof(test_data), 0,
    reinterpret_cast<struct sockaddr*>(&addr_), sizeof(addr_));
  if (n < 0) {
    RCLCPP_WARN(this->get_logger(), "Test packet send failed: %s", strerror(errno));
  } else {
    RCLCPP_INFO(this->get_logger(), "Sent test packet: 1 bool (%zd bytes) to %s:%d",
      n, target_ip_.c_str(), port_);
  }
}

void UDPSafetyGuardianSender::callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  uint8_t data = msg->data ? 1 : 0;
  ssize_t n = sendto(sock_, &data, sizeof(data), 0,
    reinterpret_cast<struct sockaddr*>(&addr_), sizeof(addr_));
  if (n < 0) {
    RCLCPP_WARN_THROTTLE(this->get_logger(),
      *this->get_clock(), 5000, "UDP send failed: %s", strerror(errno));
    return;
  }

  send_count_.fetch_add(1, std::memory_order_relaxed);
  frame_count_since_last_.fetch_add(1, std::memory_order_relaxed);
}

void UDPSafetyGuardianSender::print_stats()
{
  double now = this->now().seconds();
  int frames = frame_count_since_last_.exchange(0);
  int total = send_count_.load();

  if (frames > 0) {
    double dt = now - last_stats_time_;
    double hz = frames / dt;
    RCLCPP_INFO(this->get_logger(),
      "Rate: %.1f Hz | Throughput: %.3f MB/s | Total frames: %d | Data size: 1 bool (1 byte)",
      hz, frames * 1.0 / (1024.0 * 1024.0) / dt, total);
  } else {
    RCLCPP_INFO(this->get_logger(),
      "Total frames: %d | Data size: 1 bool (1 byte)", total);
  }
  last_stats_time_ = now;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<UDPSafetyGuardianSender>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
