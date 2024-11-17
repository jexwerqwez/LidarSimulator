#ifndef LIDAR_CONTROLLER_H
#define LIDAR_CONTROLLER_H

#include <rclcpp/rclcpp.hpp>

#include "LidarModel.h"
#include "LidarView.h"

class LidarController {
 public:
  LidarController(rclcpp::Node::SharedPtr node);

  void run();

 private:
  LidarModel model_;
  LidarView view_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::TimerBase::SharedPtr timer_;

  // функция для публикации облака точек и маркеров
  void publishData();
};

#endif  // LIDAR_CONTROLLER_H
