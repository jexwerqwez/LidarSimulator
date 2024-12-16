#include <gtest/gtest.h>

#include "../include/Lidar.h"
#include "../include/Objects/Plane.h"
#include "../include/Objects/Sphere.h"

TEST(LidarTest, RayPlaneIntersection) {
  Position3D plane_pos(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
  Plane plane(plane_pos, 10.0, 10.0);
  Eigen::Vector3d ray_origin(0.0, 0.0, 5.0);
  Eigen::Vector3d ray_direction(0.0, 0.0, -1.0);
  Point3D intersection;

  bool result = plane.intersects(ray_origin, ray_direction, 10.0, intersection);
  EXPECT_TRUE(result);
  EXPECT_NEAR(intersection.x, 0.0, 1e-3);
  EXPECT_NEAR(intersection.y, 0.0, 1e-3);
  EXPECT_NEAR(intersection.z, 0.0, 1e-6);
}

TEST(LidarTest, RayParallelToPlane) {
  Position3D plane_pos(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  Plane plane(plane_pos, 10.0, 10.0);
  Eigen::Vector3d ray_origin(0.0, 0.0, 5.0);
  Eigen::Vector3d ray_direction(1.0, 0.0, 0.0);
  Point3D intersection;

  bool result = plane.intersects(ray_origin, ray_direction, 10.0, intersection);
  EXPECT_FALSE(result);
}

TEST(LidarTest, RayOppositeToPlane) {
  Position3D plane_pos(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  Plane plane(plane_pos, 10.0, 10.0);
  Eigen::Vector3d ray_origin(0.0, 0.0, 5.0);
  Eigen::Vector3d ray_direction(0.0, 0.0, 1.0);
  Point3D intersection;

  bool result = plane.intersects(ray_origin, ray_direction, 10.0, intersection);
  EXPECT_FALSE(result);
}

TEST(LidarTest, RayHitsPlaneAtAngle) {
  Position3D plane_pos(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  Plane plane(plane_pos, 10.0, 10.0);
  Eigen::Vector3d ray_origin(0.0, 5.0, 5.0);
  Eigen::Vector3d ray_direction(0.0, -1.0, -1.0);
  Point3D intersection;

  bool result = plane.intersects(ray_origin, ray_direction.normalized(), 10.0,
                                 intersection);
  EXPECT_TRUE(result);
  EXPECT_NEAR(intersection.x, 0.0, 1e-3);
  EXPECT_NEAR(intersection.y, 0.0, 1e-3);
  EXPECT_NEAR(intersection.z, 0.0, 1e-3);
}

TEST(LidarTest, RayHitsPlaneAtOffsetAngle) {
  Position3D plane_pos(5.0, 5.0, 0.0, 0.0, 0.0, 0.0);
  Plane plane(plane_pos, 10.0, 10.0);
  Eigen::Vector3d ray_origin(10.0, 10.0, 5.0);
  Eigen::Vector3d ray_direction(-1.0, -1.0, -1.0);
  Point3D intersection;

  bool result = plane.intersects(ray_origin, ray_direction.normalized(), 20.0,
                                 intersection);
  EXPECT_TRUE(result);
  EXPECT_NEAR(intersection.x, 5.0, 1e-3);
  EXPECT_NEAR(intersection.y, 5.0, 1e-3);
  EXPECT_NEAR(intersection.z, 0.0, 1e-3);
}

TEST(LidarTest, RayHitsOffsetPlane) {
  Position3D plane_pos(10.0, 0.0, 5.0, 0.0, 0.0, 0.0);
  Plane plane(plane_pos, 10.0, 10.0);
  Eigen::Vector3d ray_origin(10.0, 5.0, 10.0);
  Eigen::Vector3d ray_direction(0.0, -1.0, -1.0);
  Point3D intersection;

  bool result = plane.intersects(ray_origin, ray_direction.normalized(), 15.0,
                                 intersection);
  EXPECT_TRUE(result);
  EXPECT_NEAR(intersection.x, 10.0, 1e-3);
  EXPECT_NEAR(intersection.y, 0.0, 1e-3);
  EXPECT_NEAR(intersection.z, 5.0, 1e-3);
}

TEST(LidarTest, RaySphereIntersection) {
  // Настройка параметров
  Position3D sphere_pos(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  Sphere sphere(sphere_pos, 1.0);
  Eigen::Vector3d ray_origin(0.0, 0.0, 5.0);
  Eigen::Vector3d ray_direction(0.0, 0.0, -1.0);
  Point3D intersection;

  // Проверка пересечения
  bool result =
      sphere.intersects(ray_origin, ray_direction, 10.0, intersection);
  EXPECT_TRUE(result);
  EXPECT_NEAR(intersection.x, 0.0, 1e-3);
  EXPECT_NEAR(intersection.y, 0.0, 1e-3);
  EXPECT_NEAR(intersection.z, 1.0, 1e-3);
}

TEST(LidarTest, RayMissesSphere) {
  Position3D sphere_pos(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  Sphere sphere(sphere_pos, 1.0);
  Eigen::Vector3d ray_origin(2.0, 2.0, 5.0);
  Eigen::Vector3d ray_direction(0.0, 0.0, -1.0);
  Point3D intersection;

  bool result =
      sphere.intersects(ray_origin, ray_direction, 10.0, intersection);
  EXPECT_FALSE(result);
}

TEST(LidarTest, RayThroughSphereCenter) {
  Position3D sphere_pos(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  Sphere sphere(sphere_pos, 1.0);
  Eigen::Vector3d ray_origin(0.0, 0.0, 5.0);
  Eigen::Vector3d ray_direction(0.0, 0.0, -1.0);
  Point3D intersection;

  bool result =
      sphere.intersects(ray_origin, ray_direction, 10.0, intersection);
  EXPECT_TRUE(result);
  EXPECT_NEAR(intersection.x, 0.0, 1e-3);
  EXPECT_NEAR(intersection.y, 0.0, 1e-3);
  EXPECT_NEAR(intersection.z, 1.0, 1e-3);
}

TEST(LidarTest, RayThroughSphereCenterFromOffset) {
  Position3D sphere_pos(3.0, 3.0, 3.0, 0.0, 0.0, 0.0);
  Sphere sphere(sphere_pos, 3.0);
  Eigen::Vector3d ray_origin(3.0, 3.0, 10.0);
  Eigen::Vector3d ray_direction(0.0, 0.0, -1.0);
  Point3D intersection;

  bool result =
      sphere.intersects(ray_origin, ray_direction, 15.0, intersection);
  EXPECT_TRUE(result);
  EXPECT_NEAR(intersection.x, 3.0, 1e-3);
  EXPECT_NEAR(intersection.y, 3.0, 1e-3);
  EXPECT_NEAR(intersection.z, 6.0, 1e-3);
}

// TEST(LidarTest, RayHitsOffsetSphere) {
//     Position3D sphere_pos(5.0, 5.0, 5.0, 0.0, 0.0, 0.0);
//     Sphere sphere(sphere_pos, 2.0);
//     Eigen::Vector3d ray_origin(10.0, 10.0, 10.0);
//     Eigen::Vector3d ray_direction(-1.0, -1.0, -1.0);
//     Point3D intersection;

//     bool result = sphere.intersects(ray_origin,
//     ray_direction.normalized(), 15.0, intersection); EXPECT_TRUE(result);
//     EXPECT_NEAR(intersection.x, 6.58, 1e-3);
//     EXPECT_NEAR(intersection.y, 6.58, 1e-3);
//     EXPECT_NEAR(intersection.z, 6.58, 1e-3);
// }

// TEST(LidarTest, RayHitsSphereAtAngle) {
//     Position3D sphere_pos(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
//     Sphere sphere(sphere_pos, 2.0);
//     Eigen::Vector3d ray_origin(0.0, -5.0, 5.0);
//     Eigen::Vector3d ray_direction(0.0, 1.0, -1.0);
//     Point3D intersection;

//     bool result = sphere.intersects(ray_origin,
//     ray_direction.normalized(), 10.0, intersection); EXPECT_TRUE(result);
//     EXPECT_NEAR(intersection.x, 0.0, 1e-3);
//     EXPECT_NEAR(intersection.y, -1.73, 1e-3);
//     EXPECT_NEAR(intersection.z, 1.73, 1e-3);
// }

// TEST(LidarTest, RayTangentToSphere) {
//     Position3D sphere_pos(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
//     Sphere sphere(sphere_pos, 1.0);
//     Eigen::Vector3d ray_origin(1.0, 0.0, 5.0);
//     Eigen::Vector3d ray_direction(0.0, 0.0, -1.0);
//     Point3D intersection;

//     bool result = sphere.intersects(ray_origin, ray_direction, 10.0,
//     intersection); EXPECT_TRUE(result); EXPECT_NEAR(intersection.x, 1.0,
//     1e-3); EXPECT_NEAR(intersection.y, 0.0, 1e-3);
//     EXPECT_NEAR(intersection.z, 1.0, 1e-3);
// }

// TEST(LidarTest, RayHitsPlaneEdge) {
//     Position3D plane_pos(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
//     Plane plane(plane_pos, 10.0, 10.0);
//     Eigen::Vector3d ray_origin(5.0, 0.0, 5.0);
//     Eigen::Vector3d ray_direction(0.0, 0.0, -1.0);
//     Point3D intersection;

//     bool result = plane.intersects(ray_origin, ray_direction, 10.0,
//     intersection); EXPECT_TRUE(result); EXPECT_NEAR(intersection.x, 5.0,
//     1e-3); EXPECT_NEAR(intersection.y, 0.0, 1e-3);
//     EXPECT_NEAR(intersection.z, 0.0, 1e-6);
// }

TEST(LidarTest, PointCloudGeneration) {
  auto node = rclcpp::Node::make_shared("test_node");
  Lidar lidar(node);
  lidar.configure(1.0, 10, -45.0, 45.0, 20.0, M_PI / 180.0);

  auto plane = std::make_shared<Plane>(Position3D(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                                       10.0, 10.0);
  lidar.addObject(plane);

  auto cloud = lidar.scan();
  EXPECT_GT(cloud->points.size(), 0);
}

TEST(LidarTest, CloudContainsPoints) {
  auto node = rclcpp::Node::make_shared("test_node");
  Lidar lidar(node);
  lidar.configure(1.0, 10, -45.0, 45.0, 20.0, M_PI / 180.0);

  auto plane = std::make_shared<Plane>(Position3D(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                                       10.0, 10.0);
  lidar.addObject(plane);

  auto cloud = lidar.scan();
  EXPECT_FALSE(cloud->points.empty());
  EXPECT_NEAR(cloud->points[0].z, 1.0, 1e-3);
}

TEST(LidarTest, MultipleObjectIntersections) {
  auto node = rclcpp::Node::make_shared("test_node");
  Lidar lidar(node);
  lidar.configure(1.0, 10, -45.0, 45.0, 20.0, M_PI / 180.0);

  auto sphere =
      std::make_shared<Sphere>(Position3D(2.0, 0.0, 1.0, 0.0, 0.0, 0.0), 1.0);
  auto plane = std::make_shared<Plane>(Position3D(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                                       10.0, 10.0);
  lidar.addObject(sphere);
  lidar.addObject(plane);

  auto cloud = lidar.scan();
  EXPECT_GT(cloud->points.size(), 1);
}

TEST(LidarTest, DynamicObjectUpdate) {
  auto node = rclcpp::Node::make_shared("test_node");
  Lidar lidar(node);
  lidar.configure(1.0, 10, -45.0, 45.0, 20.0, M_PI / 180.0);

  auto sphere =
      std::make_shared<Sphere>(Position3D(2.0, 0.0, 1.0, 0.0, 0.0, 0.0), 1.0);
  lidar.addObject(sphere);

  auto cloud_initial = lidar.scan();
  EXPECT_FALSE(cloud_initial->points.empty());

  sphere->setPosition(Position3D(4.0, 0.0, 1.0, 0.0, 0.0, 0.0));
  auto cloud_updated = lidar.scan();

  EXPECT_NE(cloud_initial->points[0].x, cloud_updated->points[0].x);
}

TEST(LidarTest, MultiLayerObjectDetection) {
  auto node = rclcpp::Node::make_shared("test_node");
  Lidar lidar(node);
  lidar.configure(1.0, 10, -45.0, 45.0, 20.0, M_PI / 180.0);

  auto plane1 = std::make_shared<Plane>(
      Position3D(0.0, 0.0, 0.0, 0.0, 0.0, 0.0), 10.0, 10.0);
  auto plane2 = std::make_shared<Plane>(
      Position3D(0.0, 0.0, 5.0, 0.0, 0.0, 0.0), 10.0, 10.0);
  lidar.addObject(plane1);
  lidar.addObject(plane2);

  auto cloud = lidar.scan();
  EXPECT_GT(cloud->points.size(), 1);
}

TEST(LidarTest, PerformanceBenchmark) {
  auto node = rclcpp::Node::make_shared("test_node");
  Lidar lidar(node);
  lidar.configure(1.0, 100, -45.0, 45.0, 50.0, M_PI / 360.0);

  auto start_time = std::chrono::high_resolution_clock::now();
  auto cloud = lidar.scan();
  auto end_time = std::chrono::high_resolution_clock::now();

  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
      end_time - start_time);
  EXPECT_LT(duration.count(), 200);
}

// TEST(LidarTest, BoundaryConditionHandling) {
//     auto node = rclcpp::Node::make_shared("test_node");
//     Lidar lidar(node);
//     lidar.configure(1.0, 10, -45.0, 45.0, 5.0, M_PI / 180.0);

//     auto plane = std::make_shared<Plane>(Position3D(0.0, 0.0, 5.1, 0.0, 0.0,
//     0.0), 10.0, 10.0); lidar.addObject(plane);

//     auto cloud = lidar.scan();
//     EXPECT_EQ(cloud->points.size(), 0);
// }

// TEST(LidarTest, BeamGenerationValidation) {
//     auto node = rclcpp::Node::make_shared("test_node");
//     Lidar lidar(node);
//     lidar.configure(1.0, 10, -45.0, 45.0, 20.0, M_PI / 180.0);

//     EXPECT_GT(lidar.getRays().size(), 0);
//     EXPECT_EQ(lidar.getRays().size(), 3600);
// }

// TEST(LidarTest, RangeAndAngleLimits) {
//     auto node = rclcpp::Node::make_shared("test_node");
//     Lidar lidar(node);
//     lidar.configure(1.0, 10, -45.0, 45.0, 5.0, M_PI / 180.0);

//     auto far_sphere = std::make_shared<Sphere>(Position3D(6.0, 0.0, 1.0, 0.0,
//     0.0, 0.0), 1.0); auto near_sphere =
//     std::make_shared<Sphere>(Position3D(2.0, 0.0, 1.0, 0.0, 0.0, 0.0), 1.0);
//     lidar.addObject(far_sphere);
//     lidar.addObject(near_sphere);

//     auto cloud = lidar.scan();
//     EXPECT_EQ(cloud->points.size(), 1);
//     EXPECT_NEAR(cloud->points[0].x, 2.0, 1e-3);
// }