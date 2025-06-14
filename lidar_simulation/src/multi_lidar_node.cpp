#include <rclcpp/rclcpp.hpp>

#include "MultiLidarSimulatorNode.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<multi_lidar_sim::MultiLidarSimulatorNode>(
      rclcpp::NodeOptions{});
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
