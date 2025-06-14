#include <gtest/gtest.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <thread>

#include "Lidar.h"
#include "Objects/Plane.h"
#include "Objects/Sphere.h"

TEST(ROS2IntegrationTest, LidarPublishesPointCloud) {
  auto node = rclcpp::Node::make_shared("test_node");

  Lidar lidar(node.get(), "test_lidar", "test_frame", "test_topic", 1.0, 10,
              -30.0, 30.0, 15.0, M_PI / 180.0);

  std::vector<std::shared_ptr<Object>> scene = {
      std::make_shared<Plane>(Position3D{0, 0, 0, 0, 0, 0}, 10, 10)};
  lidar.setPosition(Position3D{0, 0, 1, 0, 0, 0});

  sensor_msgs::msg::PointCloud2::SharedPtr received_msg = nullptr;
  auto sub = node->create_subscription<sensor_msgs::msg::PointCloud2>(
      "test_topic", 10,
      [&received_msg](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        received_msg = msg;
      });

  auto [clean, noisy] = lidar.scan(scene);
  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(*clean, msg);
  msg.header.stamp = node->now();
  msg.header.frame_id = "map";
  lidar.publishTransform();
  lidar.publishCloud(clean);

  rclcpp::spin_some(node);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  rclcpp::spin_some(node);

  ASSERT_NE(received_msg, nullptr);
  EXPECT_GT(received_msg->data.size(), 0);
}

TEST(PCLIntegrationTest, VoxelGridReducesPointCount) {
  auto node = rclcpp::Node::make_shared("pcl_test");

  Lidar lidar(node.get(), "test", "frame", "topic", 1.0, 20, -45, 45, 30.0,
              M_PI / 180.0);
  lidar.setPosition(Position3D{0, 0, 1, 0, 0, 0});

  std::vector<std::shared_ptr<Object>> scene = {
      std::make_shared<Plane>(Position3D{0, 0, 0, 0, 0, 0}, 10, 10)};

  auto [cloud, _] = lidar.scan(scene);

  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(
      new pcl::PointCloud<pcl::PointXYZI>());
  pcl::VoxelGrid<pcl::PointXYZI> voxel;
  voxel.setInputCloud(cloud);
  voxel.setLeafSize(0.2f, 0.2f, 0.2f);
  voxel.filter(*filtered);

  EXPECT_LT(filtered->points.size(), cloud->points.size());
}

TEST(StressTest, ComplexObjectMotion) {
  auto node = rclcpp::Node::make_shared("motion_test");

  Lidar lidar(node.get(), "motion_lidar", "frame", "topic", 1.0, 32, -30, 30,
              50.0, M_PI / 180.0);
  lidar.setPosition(Position3D{0, 0, 1, 0, 0, 0});

  std::vector<std::shared_ptr<Object>> scene;
  for (int i = 0; i < 100; ++i) {
    scene.push_back(std::make_shared<Sphere>(
        Position3D{0.1 * i, 0.1 * i, 1.0, 0, 0, 0}, 0.2));
  }

  for (int step = 0; step < 20; ++step) {
    for (int i = 0; i < scene.size(); ++i) {
      scene[i]->setPosition(Position3D{0.1 * i, 0.1 * step, 1.0, 0, 0, 0});
    }
    auto [cloud, _] = lidar.scan(scene);
    EXPECT_GT(cloud->points.size(), 0);
  }
}

TEST(StartupPerformanceTest, StartupCompletesQuickly) {
  auto start = std::chrono::high_resolution_clock::now();
  auto node = rclcpp::Node::make_shared("startup_test");

  Lidar lidar(node.get(), "startup_lidar", "frame", "topic", 1.0, 10, -10, 10,
              10.0, M_PI / 90.0);
  lidar.setPosition(Position3D{0, 0, 1, 0, 0, 0});

  std::vector<std::shared_ptr<Object>> scene = {
      std::make_shared<Plane>(Position3D{0, 0, 0, 0, 0, 0}, 5, 5)};

  auto [cloud, _] = lidar.scan(scene);
  auto end = std::chrono::high_resolution_clock::now();

  auto time_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(end - start)
          .count();
  EXPECT_LT(time_ms, 300);
}
