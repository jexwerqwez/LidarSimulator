#include <rclcpp/rclcpp.hpp>

#include "cloud_merger/cloud_merger_node.h"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<cloud_merger::CloudMergerNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
