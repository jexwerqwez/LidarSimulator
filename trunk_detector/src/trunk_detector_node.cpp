#include "rclcpp/rclcpp.hpp"
#include "trunk_detector/trunk_detector.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto detector = std::make_shared<TrunkDetector>();
  RCLCPP_INFO(detector->get_logger(), "trunk_detector_node started");
  rclcpp::spin(detector);
  rclcpp::shutdown();
  return 0;
}
