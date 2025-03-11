#include "../../include/Objects/Cylinder.h"

#include <cmath>

Cylinder::Cylinder(const Position3D &position, double radius, double height)
    : position_(position), radius_(radius), height_(height) {}

bool Cylinder::intersects(const Eigen::Vector3d &ray_origin,
                          const Eigen::Vector3d &ray_direction, double max_range,
                          Point3D &intersection_point) const {
  Eigen::Vector3d base_center(position_.position.x, position_.position.y, position_.position.z);
  Eigen::Vector3d top_center = base_center + Eigen::Vector3d(0, height_, 0);

  // уравнение цилиндра в 2D (без учета высоты)
  Eigen::Vector3d oc = ray_origin - base_center;
  double a = ray_direction.x() * ray_direction.x() + ray_direction.z() * ray_direction.z();
  double b = 2.0 * (oc.x() * ray_direction.x() + oc.z() * ray_direction.z());
  double c = oc.x() * oc.x() + oc.z() * oc.z() - radius_ * radius_;

  double discriminant = b * b - 4 * a * c;
  if (discriminant < 0) {
    return false;  // нет пересечения
  }

  double sqrt_disc = std::sqrt(discriminant);
  double t1 = (-b - sqrt_disc) / (2.0 * a);
  double t2 = (-b + sqrt_disc) / (2.0 * a);

  // пересекает ли луч цилиндр в пределах его высоты
  double y1 = ray_origin.y() + t1 * ray_direction.y();
  double y2 = ray_origin.y() + t2 * ray_direction.y();

  if ((y1 < base_center.y() || y1 > top_center.y()) && (y2 < base_center.y() || y2 > top_center.y())) {
    return false;  // пересечение вне высоты цилиндра
  }

  double t = (y1 >= base_center.y() && y1 <= top_center.y()) ? t1 : t2;
  if (t < 0 || t > max_range) {
    return false;  // пересечение вне диапазона
  }

  Eigen::Vector3d P = ray_origin + t * ray_direction;
  intersection_point = Point3D{P.x(), P.y(), P.z()};
  return true;
}

Position3D Cylinder::getPosition() const { return position_; }

void Cylinder::setPosition(const Position3D &position) { position_ = position; }

visualization_msgs::msg::Marker Cylinder::getMarker(int id) const {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "lidar_frame";
  marker.header.stamp = rclcpp::Clock().now();
  marker.ns = "cylinders";
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::CYLINDER;
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.pose.position.x = position_.position.x;
  marker.pose.position.y = position_.position.y;
  marker.pose.position.z = position_.position.z + height_ / 2.0; // к центру поднимаем

  marker.pose.orientation.x = position_.orientation.x();
  marker.pose.orientation.y = position_.orientation.y();
  marker.pose.orientation.z = position_.orientation.z();
  marker.pose.orientation.w = position_.orientation.w();

  marker.scale.x = radius_ * 2.0;
  marker.scale.y = radius_ * 2.0;
  marker.scale.z = height_;

  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 0.8f;

  return marker;
}
