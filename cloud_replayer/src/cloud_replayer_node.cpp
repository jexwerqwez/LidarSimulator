#include "cloud_replayer/cloud_replayer_node.h"

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <filesystem>

namespace cloud_replayer
{

CloudReplayerNode::CloudReplayerNode(const rclcpp::NodeOptions &options)
: Node("cloud_replayer_node", options)
{
  // Загрузка параметров
  this->declare_parameter<std::string>("input_file_1", "/tmp/lidar_clouds/cloud1.pcd");
  this->declare_parameter<std::string>("input_file_2", "/tmp/lidar_clouds/cloud2.pcd");
  this->declare_parameter<double>("replay_rate", 1.0);

  this->get_parameter("input_file_1", input_file_1_);
  this->get_parameter("input_file_2", input_file_2_);
  this->get_parameter("replay_rate", replay_rate_);

  // Паблишеры
  pub_cloud1_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud1", 10);
  pub_cloud2_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud2", 10);

  // Загрузка файлов
  loadPCDFiles();

  // Проверка
  if (!cloud1_loaded_ || !cloud2_loaded_) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load input clouds. Exiting.");
    return;
  }

  // Запуск таймера публикации
  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / replay_rate_),
    std::bind(&CloudReplayerNode::replayClouds, this));

  RCLCPP_INFO(this->get_logger(), "Replayer node started. Replaying two fixed clouds.");
}

void CloudReplayerNode::loadPCDFiles()
{
  if (!input_file_1_.empty() && std::filesystem::exists(input_file_1_)) {
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(input_file_1_, cloud1_) == 0) {
      cloud1_loaded_ = true;
      RCLCPP_INFO(this->get_logger(), "Loaded cloud1 from %s", input_file_1_.c_str());
    } else {
      RCLCPP_WARN(this->get_logger(), "Failed to load %s", input_file_1_.c_str());
    }
  }

  if (!input_file_2_.empty() && std::filesystem::exists(input_file_2_)) {
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(input_file_2_, cloud2_) == 0) {
      cloud2_loaded_ = true;
      RCLCPP_INFO(this->get_logger(), "Loaded cloud2 from %s", input_file_2_.c_str());
    } else {
      RCLCPP_WARN(this->get_logger(), "Failed to load %s", input_file_2_.c_str());
    }
  }
}

void CloudReplayerNode::replayClouds()
{
  if (!cloud1_loaded_ || !cloud2_loaded_) return;

  sensor_msgs::msg::PointCloud2 msg1, msg2;
  pcl::toROSMsg(cloud1_, msg1);
  pcl::toROSMsg(cloud2_, msg2);

  msg1.header.frame_id = "lidar_frame";
  msg2.header.frame_id = "lidar_frame";
  msg1.header.stamp = this->now();
  msg2.header.stamp = this->now();

  pub_cloud1_->publish(msg1);
  pub_cloud2_->publish(msg2);
}

}  // namespace cloud_replayer

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(cloud_replayer::CloudReplayerNode)
