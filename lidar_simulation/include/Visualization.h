#ifndef VISUALIZATION_H
#define VISUALIZATION_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "Objects/Object.h"

class Visualization {
 public:
  Visualization(rclcpp::Node::SharedPtr node);

  void publishPointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud);
  void publishMarkers(const std::vector<std::shared_ptr<Object>> &objects);
  void publishRays(const std::vector<Eigen::Vector3d> &rays,
                   const std::string &lidar_frame, double ray_length);

 private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
};

#endif  // VISUALIZATION_H
