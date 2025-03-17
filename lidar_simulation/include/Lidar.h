#ifndef LIDAR_H
#define LIDAR_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "Objects/Object.h"
#include "Point3D.h"
#include "Position3D.h"

class Lidar {
 public:
  Lidar(rclcpp::Node::SharedPtr node);

  void configure(double lidar_height, int num_lasers, double alpha_begin,
                 double alpha_end, double laser_range, double horizontal_step);

  void addObject(std::shared_ptr<Object> object);
  std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> scan();

  void updatePosition(double linear_velocity, double angular_velocity, double dt);
  void publishTransform();
  void publishPointCloud();
  void loadParametersFromYaml();  

 private:
  Position3D position_;
  void generateRays();

  rclcpp::Node::SharedPtr node_;
  double lidar_height_;
  int num_lasers_;
  double alpha_begin_;
  double alpha_end_;
  double laser_range_;
  double horizontal_step_;

  std::vector<Eigen::Vector3d> rays_;
  std::vector<std::shared_ptr<Object>> objects_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_publisher_;
};

#endif  // LIDAR_H
