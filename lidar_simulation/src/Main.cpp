#include "../include/LidarController.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("lidar_simulation_node");
  LidarController controller(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}