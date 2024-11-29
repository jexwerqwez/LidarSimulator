#include "../include/Visualization.h"

#include <pcl_conversions/pcl_conversions.h>

#include "../include/Objects/Plane.h"

Visualization::Visualization(rclcpp::Node::SharedPtr node) : node_(node) {
  point_cloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "lidar_points", 10);
  marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
      "visualization_marker", 10);
}

// публикация облака точек в топик
void Visualization::publishPointCloud(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud) {
  sensor_msgs::msg::PointCloud2 output;
  pcl::toROSMsg(*cloud, output);
  output.header.frame_id = "lidar_frame";
  output.header.stamp = node_->now();
  point_cloud_pub_->publish(output);
}

// публикация маркеров объектов для визуализации
void Visualization::publishMarkers(
    const std::vector<std::shared_ptr<Object>> &objects) {
  int id = 0;
  for (const auto &obj : objects) {
    auto marker = obj->getMarker(id);
    marker_pub_->publish(marker);
    id++;
  }
}
