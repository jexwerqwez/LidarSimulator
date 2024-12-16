#include <gtest/gtest.h>

#include "../include/Lidar.h"
#include "../include/Objects/Plane.h"
#include "../include/Objects/Sphere.h"

TEST(StressTest, HighObjectDensity) {
  auto node = rclcpp::Node::make_shared("test_node");
  Lidar lidar(node);
  lidar.configure(1.0, 50, -45.0, 45.0, 20.0, M_PI / 180.0);

  for (int i = 0; i < 1000; ++i) {
    auto sphere = std::make_shared<Sphere>(
        Position3D(i * 0.1, 0.0, 0.0, 0.0, 0.0, 0.0), 0.1);
    lidar.addObject(sphere);
  }

  auto start_time = std::chrono::high_resolution_clock::now();
  auto cloud = lidar.scan();
  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
      end_time - start_time);

  EXPECT_GT(cloud->points.size(), 0);
  EXPECT_LT(duration.count(), 100);
}

TEST(StressTest, HighFrequencyScanning) {
  auto node = rclcpp::Node::make_shared("test_node");
  Lidar lidar(node);
  lidar.configure(1.0, 100, -45.0, 45.0, 20.0, M_PI / 180.0);

  for (int i = 0; i < 10000; ++i) {
    auto cloud = lidar.scan();
    ASSERT_GT(cloud->points.size(), 0);
  }
}
