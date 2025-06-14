#include "../../include/Objects/Cylinder.h"

#include <cmath>

Cylinder::Cylinder(const Position3D &position, double radius, double height)
    : position_(position), radius_(radius), height_(height) {}

bool Cylinder::intersects(const Eigen::Vector3d &ray_origin,
                          const Eigen::Vector3d &ray_direction,
                          double max_range, Point3D &intersection_point) const {
  // центр основания цилиндра
  Eigen::Vector3d base_center(position_.position.x, position_.position.y,
                              position_.position.z);
  Eigen::Vector3d top_center = base_center + Eigen::Vector3d(0, 0, height_);

  // расчёт в 2D по XY-плоскости
  Eigen::Vector3d oc = ray_origin - base_center;
  double a = ray_direction.x() * ray_direction.x() +
             ray_direction.y() * ray_direction.y();
  double b = 2.0 * (oc.x() * ray_direction.x() + oc.y() * ray_direction.y());
  double c = oc.x() * oc.x() + oc.y() * oc.y() - radius_ * radius_;

  double discriminant = b * b - 4 * a * c;
  if (discriminant < 0) {
    return false;  // нет пересечения
  }

  double sqrt_disc = std::sqrt(discriminant);
  double t1 = (-b - sqrt_disc) / (2.0 * a);
  double t2 = (-b + sqrt_disc) / (2.0 * a);

  double z1 = ray_origin.z() + t1 * ray_direction.z();
  double z2 = ray_origin.z() + t2 * ray_direction.z();

  // проверяем лежит ли точка пересечения в пределах высоты цилиндра
  bool valid_z1 = z1 >= base_center.z() && z1 <= top_center.z();
  bool valid_z2 = z2 >= base_center.z() && z2 <= top_center.z();

  double t = -1.0;
  if (valid_z1 && t1 > 0 && t1 < max_range) {
    t = t1;
  } else if (valid_z2 && t2 > 0 && t2 < max_range) {
    t = t2;
  } else {
    return false;
  }

  Eigen::Vector3d P = ray_origin + t * ray_direction;
  intersection_point = Point3D{P.x(), P.y(), P.z()};
  return true;
}

Position3D Cylinder::getPosition() const { return position_; }

void Cylinder::setPosition(const Position3D &position) { position_ = position; }

visualization_msgs::msg::Marker Cylinder::getMarker(int id) const {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = rclcpp::Clock().now();
  marker.ns = "cylinders";
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::CYLINDER;
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.pose.position.x = position_.position.x;
  marker.pose.position.y = position_.position.y;
  marker.pose.position.z = position_.position.z + height_ / 2.0;  // центр по Z

  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = std::abs(radius_ * 2.0);
  marker.scale.y = std::abs(radius_ * 2.0);
  marker.scale.z = height_;

  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 0.8f;

  // marker.lifetime = rclcpp::Duration::from_seconds(0.5);

  return marker;
}
