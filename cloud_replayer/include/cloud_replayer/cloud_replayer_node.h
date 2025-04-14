#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <string>

namespace cloud_replayer
{

class CloudReplayerNode : public rclcpp::Node
{
public:
  explicit CloudReplayerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void loadPCDFiles();
  void replayClouds();

  // Параметры
  std::string cloud_directory_;
  std::string input_file_1_;
  std::string input_file_2_;
  double replay_rate_;

  // Загруженные облака
  pcl::PointCloud<pcl::PointXYZI> cloud1_;
  pcl::PointCloud<pcl::PointXYZI> cloud2_;
  bool cloud1_loaded_ = false;
  bool cloud2_loaded_ = false;

  // Публикация
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud1_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud2_;

  // Таймер для периодической публикации
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace cloud_replayer
