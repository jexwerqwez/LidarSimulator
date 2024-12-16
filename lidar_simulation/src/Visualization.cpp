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

void Visualization::publishRay(const Eigen::Vector3d &start,
                               const Eigen::Vector3d &end,
                               const std::string &ns, double width,
                               const std::array<float, 4> &color) {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "lidar_frame";
  marker.header.stamp = node_->now();
  marker.ns = ns;
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;

  geometry_msgs::msg::Point p_start, p_end;
  p_start.x = start.x();
  p_start.y = start.y();
  p_start.z = start.z();
  p_end.x = end.x();
  p_end.y = end.y();
  p_end.z = end.z();

  marker.points.push_back(p_start);
  marker.points.push_back(p_end);

  marker.scale.x = width;  // толщина линии
  marker.scale.y = 0.1;    // ширина стрелки
  marker.scale.z = 0.1;

  marker.color.r = color[0];
  marker.color.g = color[1];
  marker.color.b = color[2];
  marker.color.a = color[3];

  marker_pub_->publish(marker);
}

void Visualization::publishPlane(const Plane &plane, const std::string &ns,
                                 const std::array<float, 4> &color) {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "lidar_frame";
  marker.header.stamp = node_->now();
  marker.ns = ns;
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.action = visualization_msgs::msg::Marker::ADD;

  auto position = plane.getPosition();
  marker.pose.position.x = position.position.x;
  marker.pose.position.y = position.position.y;
  marker.pose.position.z = position.position.z;

  marker.pose.orientation.x = position.orientation.x();
  marker.pose.orientation.y = position.orientation.y();
  marker.pose.orientation.z = position.orientation.z();
  marker.pose.orientation.w = position.orientation.w();

  marker.scale.x = plane.getWidth();
  marker.scale.y = 0.01;  // минимальная толщина для визуализации
  marker.scale.z = plane.getHeight();

  marker.color.r = color[0];
  marker.color.g = color[1];
  marker.color.b = color[2];
  marker.color.a = color[3];

  marker_pub_->publish(marker);
}
