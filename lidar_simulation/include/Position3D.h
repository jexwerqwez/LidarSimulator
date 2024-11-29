#ifndef POSITION3D_H
#define POSITION3D_H

#include <Eigen/Geometry>

struct Position3D {
  Point3D position;
  Eigen::Quaterniond orientation;

  Position3D()
      : position{0.0, 0.0, 0.0}, orientation(Eigen::Quaterniond::Identity()) {}

  Position3D(double x, double y, double z, double roll, double pitch,
             double yaw) {
    position = {x, y, z};
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
    orientation = yawAngle * pitchAngle * rollAngle;
  }

  Eigen::Matrix4d getTransformationMatrix() const {
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform.block<3, 3>(0, 0) = orientation.toRotationMatrix();
    transform.block<3, 1>(0, 3) =
        Eigen::Vector3d(position.x, position.y, position.z);
    return transform;
  }
};

#endif  // POSITION3D_H
