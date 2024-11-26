#ifndef LIDAR_MODEL_H
#define LIDAR_MODEL_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <vector>

class LidarModel {
 public:
  struct Plane {
    Eigen::Vector3d normal;  // нормаль плоскости
    double height;           // высота плоскости
  };

  struct Sphere {
    Eigen::Vector3d center;  // центр сферы
    double radius;           // радиус сферы
  };

  LidarModel();
  LidarModel(rclcpp::Node::SharedPtr node);

  void configure(double lidar_height, int num_lasers, double alpha_begin,
                 double alpha_end, double laser_range,
                 const std::vector<double>& planes_tilt_angles,
                 const std::vector<double>& planes_heights,
                 const std::vector<Sphere>& spheres);

  pcl::PointCloud<pcl::PointXYZI>::Ptr getPointCloud() const;
  const std::vector<Plane>& getPlanes() const;
  const std::vector<Sphere>& getSpheres() const;

  void findIntersections();  // функция для поиска точек пересечения лучей с
                             // плоскостями

 private:
  bool intersectsSphere(const Eigen::Vector3d& ray, const Sphere& sphere,
                        Eigen::Vector3d& intersection) const;
  void generateRays();    // генерация лучей
  void generatePlanes();  // создание плоскостей
  bool isWithinBounds(const Eigen::Vector3d& point, size_t current_plane_index);

  rclcpp::Node::SharedPtr node_;
  double lidar_height_;
  int num_lasers_;
  double alpha_begin_;
  double alpha_end_;
  double laser_range_;
  std::vector<Eigen::Vector3d> rays_;
  std::vector<Plane> planes_;
  std::vector<double> planes_tilt_angles_;
  std::vector<double> planes_heights_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_;
  std::vector<Sphere> spheres_;
};

#endif  // LIDAR_MODEL_H
