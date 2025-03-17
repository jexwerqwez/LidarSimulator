#ifndef LIDAR_CONTROLLER_H
#define LIDAR_CONTROLLER_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include "Lidar.h"
#include "Objects/Object.h"
#include "Visualization.h"

class LidarController {
 public:
  LidarController(rclcpp::Node::SharedPtr node);
  void run();

 private:
  void publishData();
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

  Lidar lidar_;
  Visualization visualization_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<std::shared_ptr<Object>> objects_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  
  double linear_velocity_;
  double angular_velocity_;
};

#endif  // LIDAR_CONTROLLER_H
