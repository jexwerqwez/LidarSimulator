#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "Lidar.h"
#include "Objects/Cylinder.h"
#include "Objects/Plane.h"
#include "Objects/Sphere.h"

TEST(StressTest, HighObjectDensity) {
  auto node = rclcpp::Node::make_shared("stress_density");
  Lidar lidar(node.get(), "lidar", "frame", "topic", 1.0, 64, -30.0, 30.0, 25.0,
              M_PI / 180.0);

  lidar.setPosition(Position3D{0, 0, 1.0, 0, 0, 0});
  std::vector<std::shared_ptr<Object>> objects;

  for (int i = 0; i < 3000; ++i) {
    objects.push_back(
        std::make_shared<Sphere>(Position3D{i * 0.1, 0, 0, 0, 0, 0}, 0.1));
  }

  auto [cloud, _] = lidar.scan(objects);
  EXPECT_GT(cloud->points.size(), 100);
}

TEST(StressTest, HighFrequencyScanning) {
  auto node = rclcpp::Node::make_shared("stress_frequency");
  Lidar lidar(node.get(), "lidar", "frame", "topic", 1.0, 32, -30.0, 30.0, 20.0,
              M_PI / 180.0);

  lidar.setPosition(Position3D{0, 0, 1.0, 0, 0, 0});
  std::vector<std::shared_ptr<Object>> objects = {
      std::make_shared<Plane>(Position3D{0, 0, 0, 0, 0, 0}, 20, 20)};

  for (int i = 0; i < 100; ++i) {
    auto [cloud, _] = lidar.scan(objects);
    ASSERT_GT(cloud->points.size(), 0);
  }
}

TEST(StressTest, LongTermStability) {
  auto node = rclcpp::Node::make_shared("stress_long_term");
  Lidar lidar(node.get(), "lidar", "frame", "topic", 1.0, 16, -20.0, 20.0, 15.0,
              M_PI / 180.0);

  lidar.setPosition(Position3D{0, 0, 1.0, 0, 0, 0});
  auto plane = std::make_shared<Plane>(Position3D{0, 0, 0, 0, 0, 0}, 10, 10);
  std::vector<std::shared_ptr<Object>> objects = {plane};

  for (int i = 0; i < 500; ++i) {
    auto [cloud, _] = lidar.scan(objects);
    ASSERT_FALSE(cloud->points.empty());
  }
}

TEST(StressTest, LidarColdStartTime) {
  auto start_time = std::chrono::high_resolution_clock::now();
  auto node = rclcpp::Node::make_shared("cold_start");

  Lidar lidar(node.get(), "lidar", "frame", "topic", 1.0, 16, -20.0, 20.0, 15.0,
              M_PI / 180.0);
  lidar.setPosition(Position3D{0, 0, 1.0, 0, 0, 0});
  auto end_time = std::chrono::high_resolution_clock::now();

  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(end_time -
                                                                  start_time)
                .count();
  EXPECT_LT(ms, 20);
}
