#ifndef SPHERE_H
#define SPHERE_H

#include "Object.h"

class Sphere : public Object {
 public:
  Sphere(const Position3D &position, double radius);

  bool intersects(const Eigen::Vector3d &ray_origin,
                  const Eigen::Vector3d &ray_direction, double max_range,
                  Point3D &intersection_point) const override;

  Position3D getPosition() const override;
  void setPosition(const Position3D &position) override;

  visualization_msgs::msg::Marker getMarker(int id) const override;

  double getRadius() const { return radius_; }

 private:
  Position3D position_;
  double radius_;
};

#endif  // SPHERE_H
