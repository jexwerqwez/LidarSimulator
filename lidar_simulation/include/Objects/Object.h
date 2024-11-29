#ifndef OBJECT_H
#define OBJECT_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "../Point3D.h"
#include "../Position3D.h"

// базовый класс для объектов
class Object {
 public:
  virtual ~Object() = default;

  // проверка пересечения луча с объектом
  virtual bool intersects(const Eigen::Vector3d &ray_origin,
                          const Eigen::Vector3d &ray_direction,
                          double max_range,
                          Point3D &intersection_point) const = 0;

  // получение позиции объекта в глобальной СК
  virtual Position3D getPosition() const = 0;

  // установка позиции объекта в глобальной СК
  virtual void setPosition(const Position3D &position) = 0;

  // визуализация объекта
  virtual visualization_msgs::msg::Marker getMarker(int id) const = 0;
};

#endif  // OBJECT_H
