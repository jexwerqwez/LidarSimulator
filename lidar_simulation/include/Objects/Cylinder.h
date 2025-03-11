#ifndef CYLINDER_H
#define CYLINDER_H

#include "Object.h"

class Cylinder : public Object {
 public:
  Cylinder(const Position3D &position, double radius, double height);

  bool intersects(const Eigen::Vector3d &ray_origin,
                  const Eigen::Vector3d &ray_direction, double max_range,
                  Point3D &intersection_point) const override;

  Position3D getPosition() const override;
  void setPosition(const Position3D &position) override;

  visualization_msgs::msg::Marker getMarker(int id) const override;

  double getRadius() const { return radius_; }
  double getHeight() const { return height_; }

 private:
  Position3D position_;
  double radius_;
  double height_;
};

#endif  // CYLINDER_H
