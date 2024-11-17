#include "../include/LidarModel.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>
#include <cmath>
#include <rclcpp/rclcpp.hpp>

LidarModel::LidarModel(rclcpp::Node::SharedPtr node)
    : node_(node), cloud_(new pcl::PointCloud<pcl::PointXYZI>()) {}

LidarModel::LidarModel() {
  // Реализация конструктора (может быть пустым, если не требуется)
}

void LidarModel::configure(double lidar_height, int num_lasers,
                           double alpha_begin, double alpha_end,
                           double laser_range,
                           const std::vector<double>& planes_tilt_angles,
                           const std::vector<double>& planes_heights) {
  lidar_height_ = lidar_height;
  num_lasers_ = num_lasers;
  alpha_begin_ = alpha_begin;
  alpha_end_ = alpha_end;
  laser_range_ = laser_range;
  planes_tilt_angles_ = planes_tilt_angles;
  planes_heights_ = planes_heights;

  generateRays();
  generatePlanes();
}

// функция для генерации лучей с учетом заданных углов
void LidarModel::generateRays() {
  // переводим углы в радианы и рассчитываем шаг между лазерами
  double alpha_begin_rad = alpha_begin_ * M_PI / 180.0;
  double alpha_end_rad = alpha_end_ * M_PI / 180.0;
  double vertical_angle_step =
      (alpha_end_rad - alpha_begin_rad) / (num_lasers_ - 1);
  // циклы для создания всех лучей, проходящих вокруг по горизонтали и с
  // заданными вертикальными углами
  for (double yaw = 0.0; yaw < 2 * M_PI;
       yaw += (2 * M_PI) /
              360.0) {  // проходим круг по горизонтали с шагом в один градус
    for (int i = 0; i < num_lasers_;
         ++i) {  // проходим по всем лазерным вертикальным углам
      double vertical_angle =
          alpha_begin_rad +
          i * vertical_angle_step;  // высчитываем текущий вертикальный угол
      rays_.emplace_back(
          cos(vertical_angle) *
              cos(yaw),  // координата x с учетом вертикального и
                         // горизонтального угла
          cos(vertical_angle) *
              sin(yaw),  // координата y с учетом вертикального и
                         // горизонтального угла
          sin(vertical_angle)  // координата z по вертикальному углу
      );
    }
  }
}

// функция для создания наклонных плоскостей
void LidarModel::generatePlanes() {
  // проверяем, совпадает ли количество углов с количеством высот
  if (planes_tilt_angles_.size() != planes_heights_.size()) {
    RCLCPP_ERROR(node_->get_logger(),
                 "Количество углов наклона не соответствует количеству высот "
                 "плоскостей.");
    return;
  }

  // создаем каждую плоскость, вычисляя нормаль и высоту
  for (size_t i = 0; i < planes_tilt_angles_.size(); ++i) {
    double tilt_angle_rad = planes_tilt_angles_[i] * M_PI / 180.0;
    Plane plane;
    plane.normal = Eigen::Vector3d(sin(tilt_angle_rad), 0, cos(tilt_angle_rad));
    plane.height = planes_heights_[i];
    planes_.push_back(plane);
  }
}

// функция для поиска точек пересечения лучей с плоскостями
void LidarModel::findIntersections() {
  cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());

  // для каждого луча ищем ближайшую точку пересечения
  for (const auto& ray : rays_) {
    Eigen::Vector3d nearest_intersection;
    double min_distance = laser_range_;

    for (size_t i = 0; i < planes_.size(); ++i) {
      const auto& plane = planes_[i];
      double denominator = ray.dot(plane.normal);

      if (fabs(denominator) >
          1e-6) {  // проверяем, что луч не параллелен плоскости
        double t = -plane.height / denominator;

        if (t > 0 && t <= laser_range_) {
          Eigen::Vector3d intersection_point = t * ray;

          if (isWithinBounds(intersection_point, i)) {
            double distance = intersection_point.norm();

            if (distance < min_distance) {
              min_distance = distance;
              nearest_intersection = intersection_point;
            }
          }
        }
      }
    }

    // сохраняем ближайшую точку в облако точек
    if (min_distance < laser_range_) {
      pcl::PointXYZI point;
      point.x = nearest_intersection.x();
      point.y = nearest_intersection.y();
      point.z = nearest_intersection.z();
      point.intensity = min_distance;
      cloud_->points.push_back(point);
    }
  }

  // настраиваем параметры облака точек
  cloud_->width = static_cast<uint32_t>(cloud_->points.size());
  cloud_->height = 1;
  cloud_->is_dense = false;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr LidarModel::getPointCloud() const {
  return cloud_;
}

const std::vector<LidarModel::Plane>& LidarModel::getPlanes() const {
  return planes_;
}

// функция для проверки, находится ли точка внутри допустимых границ
bool LidarModel::isWithinBounds(const Eigen::Vector3d& point,
                                size_t current_plane_index) {
  for (size_t i = 0; i < planes_.size(); ++i) {
    if (i == current_plane_index) continue;

    const auto& other_plane = planes_[i];
    double side = point.dot(other_plane.normal) + other_plane.height;

    if (side > 0) {  // точка выходит за пределы плоскости
      return false;
    }
  }

  return true;
}
