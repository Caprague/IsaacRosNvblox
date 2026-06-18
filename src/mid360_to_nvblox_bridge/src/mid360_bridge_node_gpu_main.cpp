// SPDX-License-Identifier: Apache-2.0
// Standalone executable for Mid360 Bridge Node (GPU)

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "mid360_to_nvblox_bridge/mid360_bridge_node_gpu.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  auto node = std::make_shared<mid360_bridge::Mid360BridgeNode>(options);

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
