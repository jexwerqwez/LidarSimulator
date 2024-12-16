#ifndef LIDAR_H
#define LIDAR_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

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

  pcl::PointCloud<pcl::PointXYZI>::Ptr scan();

  std::vector<Eigen::Vector3d> getRays() const; 

 private:
  void generateRays();

  rclcpp::Node::SharedPtr node_;
  double lidar_height_;
  int num_lasers_;
  double alpha_begin_;
  double alpha_end_;
  double laser_range_;
  double horizontal_step_;  // шаг по горизонтали в радианах

  std::vector<Eigen::Vector3d> rays_;  // лучи в локальной СК лидара
  std::vector<std::shared_ptr<Object>> objects_;
};

#endif  // LIDAR_H
