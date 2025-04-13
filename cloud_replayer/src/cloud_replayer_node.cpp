#include "../include/cloud_replayer/cloud_replayer_node.h"
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <chrono>
#include <algorithm>

namespace cloud_replayer
{

CloudReplayerNode::CloudReplayerNode(const rclcpp::NodeOptions &options)
  : Node("cloud_replayer_node", options), current_index_(0)
{
  this->declare_parameter("cloud_directory", "/tmp/lidar_clouds");
  this->declare_parameter("replay_rate", 1.0);

  this->get_parameter("cloud_directory", cloud_directory_);
  this->get_parameter("replay_rate", replay_rate_);

  pub_cloud1_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud1", 10);
  pub_cloud2_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud2", 10);

  loadPCDFilenames();

  if (pcd_files_.size() < 2) {
    RCLCPP_WARN(this->get_logger(), "Not enough .pcd files in %s", cloud_directory_.c_str());
  }

  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / replay_rate_),
    std::bind(&CloudReplayerNode::replayClouds, this));

  RCLCPP_INFO(this->get_logger(), "CloudReplayerNode ready. Found %zu .pcd files", pcd_files_.size());
}

void CloudReplayerNode::loadPCDFilenames()
{
  pcd_files_.clear();
  for (const auto &entry : std::filesystem::directory_iterator(cloud_directory_)) {
    if (entry.path().extension() == ".pcd") {
      pcd_files_.push_back(entry.path().string());
    }
  }
  std::sort(pcd_files_.begin(), pcd_files_.end());  // сортировка по имени
}

void CloudReplayerNode::replayClouds()
{
  if (pcd_files_.size() < 2) return;

  size_t idx1 = current_index_;
  size_t idx2 = (current_index_ + 1) % pcd_files_.size();

  pcl::PointCloud<pcl::PointXYZI> cloud1, cloud2;
  if (pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_files_[idx1], cloud1) == -1 ||
      pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_files_[idx2], cloud2) == -1) {
    RCLCPP_WARN(this->get_logger(), "Failed to load PCD files %s or %s", 
                pcd_files_[idx1].c_str(), pcd_files_[idx2].c_str());
    return;
  }

  sensor_msgs::msg::PointCloud2 msg1, msg2;
  pcl::toROSMsg(cloud1, msg1);
  pcl::toROSMsg(cloud2, msg2);

  msg1.header.frame_id = "lidar_frame";
  msg2.header.frame_id = "lidar_frame";
  msg1.header.stamp = this->now();
  msg2.header.stamp = this->now();

  pub_cloud1_->publish(msg1);
  pub_cloud2_->publish(msg2);

  current_index_ = (current_index_ + 1) % pcd_files_.size();
}

}  // namespace cloud_replayer

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(cloud_replayer::CloudReplayerNode)
