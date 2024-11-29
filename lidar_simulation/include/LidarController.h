#ifndef LIDAR_CONTROLLER_H
#define LIDAR_CONTROLLER_H

#include <rclcpp/rclcpp.hpp>

#include "Lidar.h"
#include "Objects/Object.h"
#include "Visualization.h"

class LidarController {
 public:
  LidarController(rclcpp::Node::SharedPtr node);

  void run();

 private:
  void publishData();

  Lidar lidar_;
  Visualization visualization_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<std::shared_ptr<Object>> objects_;
};

#endif  // LIDAR_CONTROLLER_H
