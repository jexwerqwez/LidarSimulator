#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <vector>
#include <filesystem>

namespace cloud_replayer
{

class CloudReplayerNode : public rclcpp::Node
{
public:
  explicit CloudReplayerNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  void loadPCDFilenames();
  void replayClouds();

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud1_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud2_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string cloud_directory_;
  std::vector<std::string> pcd_files_;
  size_t current_index_;
  double replay_rate_;
};

}  // namespace cloud_replayer
