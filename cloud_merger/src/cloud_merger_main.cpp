#include "cloud_merger/cloud_merger_node.h"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // Создаём и запускаем узел
  auto node = std::make_shared<cloud_merger::CloudMergerNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
