#include "../../include/Objects/Plane.h"

#include <Eigen/Geometry>
#include <cmath>

Plane::Plane(const Position3D &position, double width, double height)
    : position_(position), width_(width), height_(height) {}

bool Plane::intersects(const Eigen::Vector3d &ray_origin,
                       const Eigen::Vector3d &ray_direction, double max_range,
                       Point3D &intersection_point) const {
  Eigen::Vector3d normal =
      position_.orientation.toRotationMatrix() * Eigen::Vector3d::UnitZ();

  double d = -normal.dot(Eigen::Vector3d(
      position_.position.x, position_.position.y, position_.position.z));

  double denom = ray_direction.dot(normal);
  if (std::abs(denom) < 1e-6) {
    return false;  // луч параллелен плоскости
  }

  double t = -(normal.dot(ray_origin) + d) / denom;
  if (t < 0 || t > max_range) {
    return false;  // пересечение вне диапазона
  }

  Eigen::Vector3d P = ray_origin + t * ray_direction;

  // проверяем находится ли точка внутри размеров плоскости
  Eigen::Vector3d local =
      position_.orientation.inverse() *
      (P - Eigen::Vector3d(position_.position.x, position_.position.y,
                           position_.position.z));
  if (std::abs(local.x()) > width_ / 2.0 ||
      std::abs(local.y()) > height_ / 2.0) {
    return false;
  }

  intersection_point = Point3D{P.x(), P.y(), P.z()};
  return true;
}

Position3D Plane::getPosition() const { return position_; }

double Plane::getHeight() const { return height_; }

double Plane::getWidth() const { return width_; }

void Plane::setPosition(const Position3D &position) { position_ = position; }

visualization_msgs::msg::Marker Plane::getMarker(int id) const {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = rclcpp::Clock().now();
  marker.ns = "planes";
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.pose.position.x = position_.position.x;
  marker.pose.position.y = position_.position.y;
  marker.pose.position.z = position_.position.z;
  marker.pose.orientation.x = position_.orientation.x();
  marker.pose.orientation.y = position_.orientation.y();
  marker.pose.orientation.z = position_.orientation.z();
  marker.pose.orientation.w = position_.orientation.w();

  marker.scale.x = width_;
  marker.scale.z = 0.01;
  marker.scale.y = height_;

  marker.color.r = 0.0f;
  marker.color.g = 0.0f;
  marker.color.b = 1.0f;
  marker.color.a = 0.5f;

  return marker;
}
