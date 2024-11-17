#include "../include/LidarView.h"

LidarView::LidarView(rclcpp::Node::SharedPtr node) : node_(node) {
  point_cloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "lidar_points", 10);
  marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
      "visualization_marker", 10);
}

// публикация облака точек в топик
void LidarView::publishPointCloud(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {
  sensor_msgs::msg::PointCloud2 output;
  pcl::toROSMsg(*cloud, output);
  output.header.frame_id = "lidar_frame";
  output.header.stamp = node_->now();
  point_cloud_pub_->publish(output);
}

// публикация маркеров плоскостей для визуализации
void LidarView::publishPlaneMarkers(
    const std::vector<LidarModel::Plane>& planes,
    const std::string& lidar_frame) {
  for (size_t i = 0; i < planes.size(); ++i) {
    visualization_msgs::msg::Marker plane_marker;
    plane_marker.header.frame_id = lidar_frame;
    plane_marker.header.stamp = node_->now();
    plane_marker.ns = "lidar_plane";
    plane_marker.id = static_cast<int>(i);
    plane_marker.type = visualization_msgs::msg::Marker::CUBE;
    plane_marker.action = visualization_msgs::msg::Marker::ADD;

    plane_marker.pose.position.x = 0.0;
    plane_marker.pose.position.y = 0.0;
    plane_marker.pose.position.z = planes[i].height / 2.0;

    tf2::Quaternion orientation;
    orientation.setRPY(0, asin(planes[i].normal.x()), 0);
    plane_marker.pose.orientation.x = orientation.x();
    plane_marker.pose.orientation.y = orientation.y();
    plane_marker.pose.orientation.z = orientation.z();
    plane_marker.pose.orientation.w = orientation.w();

    plane_marker.scale.x = 10.0;
    plane_marker.scale.y = 10.0;
    plane_marker.scale.z = 0.01;
    plane_marker.color.r = 0.0f;
    plane_marker.color.g = 0.0f;
    plane_marker.color.b = 1.0f;
    plane_marker.color.a = 0.5f;

    marker_pub_->publish(plane_marker);
  }
}

void LidarView::publishSphereMarkers(
    const std::vector<LidarModel::Sphere>& spheres,
    const std::string& lidar_frame) {
  for (size_t i = 0; i < spheres.size(); ++i) {
    visualization_msgs::msg::Marker sphere_marker;
    sphere_marker.header.frame_id = lidar_frame;
    sphere_marker.header.stamp = node_->now();
    sphere_marker.ns = "lidar_sphere";
    sphere_marker.id = static_cast<int>(i);
    sphere_marker.type = visualization_msgs::msg::Marker::SPHERE;
    sphere_marker.action = visualization_msgs::msg::Marker::ADD;

    sphere_marker.pose.position.x = spheres[i].center.x();
    sphere_marker.pose.position.y = spheres[i].center.y();
    sphere_marker.pose.position.z = spheres[i].center.z();
    sphere_marker.pose.orientation.w = 1.0;

    sphere_marker.scale.x = spheres[i].radius * 2.0;
    sphere_marker.scale.y = spheres[i].radius * 2.0;
    sphere_marker.scale.z = spheres[i].radius * 2.0;

    sphere_marker.color.r = 1.0f;
    sphere_marker.color.g = 0.0f;
    sphere_marker.color.b = 0.0f;
    sphere_marker.color.a = 0.8f;

    marker_pub_->publish(sphere_marker);
  }
}
