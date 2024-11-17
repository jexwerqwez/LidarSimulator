#ifndef LIDAR_VIEW_H
#define LIDAR_VIEW_H

#include <pcl_conversions/pcl_conversions.h>
#include <tf2/LinearMath/Quaternion.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "LidarModel.h"

class LidarView {
 public:
  explicit LidarView(rclcpp::Node::SharedPtr node);

  void publishPointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
  void publishPlaneMarkers(const std::vector<LidarModel::Plane>& planes,
                           const std::string& lidar_frame);

 private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
};

#endif  // LIDAR_VIEW_H
