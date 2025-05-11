#include "../../include/Objects/Sphere.h"

#include <cmath>

Sphere::Sphere(const Position3D &position, double radius)
    : position_(position), radius_(radius) {}

bool Sphere::intersects(const Eigen::Vector3d &ray_origin,
                        const Eigen::Vector3d &ray_direction, double max_range,
                        Point3D &intersection_point) const {
  Eigen::Vector3d oc =
      ray_origin - Eigen::Vector3d(position_.position.x, position_.position.y,
                                   position_.position.z);
  double a = ray_direction.dot(ray_direction);
  double b = 2.0 * oc.dot(ray_direction);
  double c = oc.dot(oc) - radius_ * radius_;

  double discriminant = b * b - 4 * a * c;
  if (discriminant < 0) {
    return false;  // нет пересечения
  }

  double sqrt_disc = std::sqrt(discriminant);
  double t1 = (-b - sqrt_disc) / (2.0 * a);
  double t2 = (-b + sqrt_disc) / (2.0 * a);

  double t = t1;
  if (t < 0 || t > max_range) {
    t = t2;
    if (t < 0 || t > max_range) {
      return false;  // пересечение вне диапазона
    }
  }

  Eigen::Vector3d P = ray_origin + t * ray_direction;
  intersection_point = Point3D{P.x(), P.y(), P.z()};
  return true;
}

Position3D Sphere::getPosition() const { return position_; }

void Sphere::setPosition(const Position3D &position) { position_ = position; }

visualization_msgs::msg::Marker Sphere::getMarker(int id) const {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = rclcpp::Clock().now();
  marker.ns = "spheres";
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.pose.position.x = position_.position.x;
  marker.pose.position.y = position_.position.y;
  marker.pose.position.z = position_.position.z;
  marker.pose.orientation.x = position_.orientation.x();
  marker.pose.orientation.y = position_.orientation.y();
  marker.pose.orientation.z = position_.orientation.z();
  marker.pose.orientation.w = position_.orientation.w();

  marker.scale.x = std::abs(radius_ * 2.0);
  marker.scale.y = std::abs(radius_ * 2.0);
  marker.scale.z = std::abs(radius_ * 2.0);

  marker.color.r = 1.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;
  marker.color.a = 0.5f;

  return marker;
}
