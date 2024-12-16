#ifndef PLANE_H
#define PLANE_H

#include "Object.h"

class Plane : public Object {
 public:
  Plane(const Position3D &position, double width, double height);

  bool intersects(const Eigen::Vector3d &ray_origin,
                  const Eigen::Vector3d &ray_direction, double max_range,
                  Point3D &intersection_point) const override;

  Position3D getPosition() const override;
  double getWidth() const;
  double getHeight() const;
  void setPosition(const Position3D &position) override;

  visualization_msgs::msg::Marker getMarker(int id) const override;

 private:
  Position3D position_;
  double width_;
  double height_;
};

#endif  // PLANE_H
