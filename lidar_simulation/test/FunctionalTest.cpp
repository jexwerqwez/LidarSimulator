#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <vector>

#include "Lidar.h"
#include "Objects/Plane.h"
#include "Objects/Sphere.h"
#include "Objects/Cylinder.h"

TEST(LidarTest, GeneratesNonEmptyPointCloud) {
  auto node = rclcpp::Node::make_shared("test_node");

  Lidar lidar(node.get(), "test_lidar", "test_frame", "test_topic",
              1.0, 16, -30.0, 10.0, 20.0, M_PI / 180.0);

  lidar.setPosition(Position3D{0, 0, 1, 0, 0, 0});

  std::vector<std::shared_ptr<Object>> objects;
  objects.push_back(std::make_shared<Plane>(Position3D{0, 0, 0, 0, 0, 0}, 10.0, 10.0));

  auto [cloud, noisy] = lidar.scan(objects);
  EXPECT_GT(cloud->points.size(), 0);
  EXPECT_GT(noisy->points.size(), 0);
}

TEST(LidarTest, DetectsMultipleObjects) {
  auto node = rclcpp::Node::make_shared("test_node");

  Lidar lidar(node.get(), "lidar", "frame", "topic",
              1.0, 32, -45.0, 45.0, 25.0, M_PI / 90.0);

  lidar.setPosition(Position3D{0, 0, 2, 0, 0, 0});

  std::vector<std::shared_ptr<Object>> scene = {
    std::make_shared<Plane>(Position3D{0, 0, 0, 0, 0, 0}, 8.0, 8.0),
    std::make_shared<Sphere>(Position3D{2.0, 0.0, 1.0, 0, 0, 0}, 0.5),
    std::make_shared<Cylinder>(Position3D{-1.0, 2.0, 0.0, 0, 0, 0}, 1.0, 1.0)
  };

  auto [cloud, _] = lidar.scan(scene);
  EXPECT_GE(cloud->points.size(), 3);
}

TEST(LidarTest, IgnoresObjectsOutsideRange) {
  auto node = rclcpp::Node::make_shared("test_node");

  Lidar lidar(node.get(), "lidar", "frame", "topic",
              1.0, 32, -45.0, 45.0, 3.0, M_PI / 180.0);  // only up to 3m

  lidar.setPosition(Position3D{0, 0, 1, 0, 0, 0});

  std::vector<std::shared_ptr<Object>> objects = {
    std::make_shared<Sphere>(Position3D{10.0, 0.0, 1.0, 0, 0, 0}, 1.0)
  };

  auto [cloud, _] = lidar.scan(objects);
  EXPECT_EQ(cloud->points.size(), 0);
}

TEST(LidarTest, DetectsMovedObject) {
  auto node = rclcpp::Node::make_shared("test_node");

  Lidar lidar(node.get(), "lidar", "frame", "topic",
              1.0, 16, -20.0, 20.0, 10.0, M_PI / 90.0);

  lidar.setPosition(Position3D{0, 0, 1, 0, 0, 0});

  auto sphere = std::make_shared<Sphere>(Position3D{0, 2.0, 1.0, 0, 0, 0}, 0.5);

  std::vector<std::shared_ptr<Object>> objects = { sphere };
  auto [cloud_before, _] = lidar.scan(objects);

  sphere->setPosition(Position3D{0, 4.0, 1.0, 0, 0, 0});
  auto [cloud_after, __] = lidar.scan(objects);

  ASSERT_FALSE(cloud_before->points.empty());
  ASSERT_FALSE(cloud_after->points.empty());
  EXPECT_NEAR(cloud_before->points[0].y, 2.0, 0.5);
  EXPECT_NEAR(cloud_after->points[0].y, 4.0, 0.5);
}

TEST(LidarTest, DetectsLayeredPlanes) {
  auto node = rclcpp::Node::make_shared("test_node");

  Lidar lidar(node.get(), "lidar", "frame", "topic",
              1.0, 20, -30.0, 30.0, 15.0, M_PI / 180.0);

  lidar.setPosition(Position3D{0, 0, 3, 0, 0, 0});

  std::vector<std::shared_ptr<Object>> objects = {
    std::make_shared<Plane>(Position3D{0, 0, 0, 0, 0, 0}, 10.0, 10.0),
    std::make_shared<Plane>(Position3D{0, 0, 5.0, 0, 0, 0}, 10.0, 10.0)
  };

  auto [cloud, _] = lidar.scan(objects);
  EXPECT_GT(cloud->points.size(), 10);
}

// TEST(LidarTest, RunsWithinReasonableTime) {
//   auto node = rclcpp::Node::make_shared("test_node");

//   Lidar lidar(node.get(), "lidar", "frame", "topic",
//               1.0, 64, -45.0, 45.0, 30.0, M_PI / 360.0);  // high density

//   lidar.setPosition(Position3D{0, 0, 1, 0, 0, 0});

//   std::vector<std::shared_ptr<Object>> objects = {
//     std::make_shared<Plane>(Position3D{0, 0, 0, 0, 0, 0}, 10.0, 10.0)
//   };

//   auto start = std::chrono::high_resolution_clock::now();
//   auto [cloud, _] = lidar.scan(objects);
//   auto end = std::chrono::high_resolution_clock::now();

//   auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
//   EXPECT_LT(ms, 150);  // < 150ms
//   EXPECT_GT(cloud->points.size(), 100);
// }

TEST(LidarFunctionalTest, EmptySceneProducesNoPoints) {
  auto node = rclcpp::Node::make_shared("test_empty_scene");
  Lidar lidar(node.get(), "test_lidar", "test_frame", "test_topic", 1.0,
              10, -30.0, 30.0, 10.0, M_PI / 180.0);

  std::vector<std::shared_ptr<Object>> empty_scene;
  auto [clean, noisy] = lidar.scan(empty_scene);

  EXPECT_EQ(clean->points.size(), 0);
  EXPECT_EQ(noisy->points.size(), 0);
}

TEST(LidarFunctionalTest, SingleSphereHit) {
  auto node = rclcpp::Node::make_shared("test_single_hit");
  Lidar lidar(node.get(), "test_lidar", "test_frame", "test_topic", 1.0,
              5, -10.0, 10.0, 15.0, M_PI / 60.0);

  std::vector<std::shared_ptr<Object>> scene = {
    std::make_shared<Sphere>(Position3D{2.0, 0.0, 1.0, 0, 0, 0}, 1.0)
  };

  auto [clean, noisy] = lidar.scan(scene);
  EXPECT_GT(clean->points.size(), 0);
  EXPECT_GT(noisy->points.size(), 0);
}

// TEST(LidarFunctionalTest, NearestObjectIsDetected) {
//   auto node = rclcpp::Node::make_shared("test_nearest");
//   Lidar lidar(node.get(), "test_lidar", "test_frame", "test_topic", 1.0,
//               5, -10.0, 10.0, 20.0, M_PI / 60.0);

//   auto near_obj = std::make_shared<Sphere>(Position3D{2.0, 0.0, 1.0, 0, 0, 0}, 0.5);
//   auto far_obj  = std::make_shared<Sphere>(Position3D{6.0, 0.0, 1.0, 0, 0, 0}, 0.5);

//   std::vector<std::shared_ptr<Object>> scene = {far_obj, near_obj};

//   auto [clean, noisy] = lidar.scan(scene);

//   ASSERT_GT(clean->points.size(), 0);
//   double min_x = clean->points[0].x;
//   for (const auto& pt : clean->points) {
//     EXPECT_LE(pt.x, 4.0);  // ближний объект должен быть ближе 4.0
//   }
// }

TEST(LidarFunctionalTest, LidarOrientationChangesScan) {
  auto node = rclcpp::Node::make_shared("test_orientation");
  Lidar lidar(node.get(), "test_lidar", "test_frame", "test_topic", 1.0,
              10, -30.0, 30.0, 15.0, M_PI / 180.0);

  std::vector<std::shared_ptr<Object>> scene = {
    std::make_shared<Plane>(Position3D{0, 0, 0, 0, 0, 0}, 10, 10)
  };

  Position3D pose1{0, 0, 1.0, 0, 0, 0};  // Прямо вниз
  Position3D pose2{0, 0, 1.0, 0, M_PI / 6, 0};  // Наклон

  lidar.setPosition(pose1);
  auto [cloud1, _] = lidar.scan(scene);

  lidar.setPosition(pose2);
  auto [cloud2, __] = lidar.scan(scene);

  EXPECT_NE(cloud1->points.size(), cloud2->points.size());
}

// TEST(LidarFunctionalTest, NoisyCloudDiffersFromClean) {
//   auto node = rclcpp::Node::make_shared("test_noise");
//   Lidar lidar(node.get(), "test_lidar", "test_frame", "test_topic", 1.0,
//               5, -10.0, 10.0, 15.0, M_PI / 60.0);

//   std::vector<std::shared_ptr<Object>> scene = {
//     std::make_shared<Plane>(Position3D{0.0, 0.0, 0.0, 0, 0, 0}, 10.0, 10.0)
//   };

//   auto [clean, noisy] = lidar.scan(scene);

//   ASSERT_EQ(clean->points.size(), noisy->points.size());
//   bool different = false;
//   for (size_t i = 0; i < clean->points.size(); ++i) {
//     if (std::abs(clean->points[i].x - noisy->points[i].x) > 1e-4 ||
//         std::abs(clean->points[i].y - noisy->points[i].y) > 1e-4 ||
//         std::abs(clean->points[i].z - noisy->points[i].z) > 1e-4) {
//       different = true;
//       break;
//     }
//   }
//   EXPECT_TRUE(different);
// }