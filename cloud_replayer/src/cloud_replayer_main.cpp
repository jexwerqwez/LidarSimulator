#include <rclcpp/rclcpp.hpp>
#include "cloud_replayer/cloud_replayer_node.h"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<cloud_replayer::CloudReplayerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
