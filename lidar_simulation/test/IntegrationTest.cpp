#include <gtest/gtest.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "../include/Lidar.h"
#include "../include/Objects/Plane.h"
#include "../include/Objects/Sphere.h"

TEST(ROS2IntegrationTest, LidarPublishesPointCloud) {
  auto node = rclcpp::Node::make_shared("test_node");
  auto lidar = std::make_shared<Lidar>(node);
  lidar->configure(1.0, 10, -45.0, 45.0, 20.0, M_PI / 180.0);

  auto subscription = node->create_subscription<sensor_msgs::msg::PointCloud2>(
      "lidar_points", 10, [](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        ASSERT_GT(msg->data.size(), 0);  // облако точек публикуется
      });

  auto timer = node->create_wall_timer(std::chrono::milliseconds(100),
                                       [&lidar]() { lidar->scan(); });

  rclcpp::spin_some(node);
}

// TEST(PCLIntegrationTest, VoxelGridFilterReducesPoints) {
//     auto node = rclcpp::Node::make_shared("test_node");
//     Lidar lidar(node);
//     lidar.configure(1.0, 10, -45.0, 45.0, 20.0, M_PI / 180.0);

//     auto cloud = lidar.scan();

//     pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new
//     pcl::PointCloud<pcl::PointXYZI>()); pcl::VoxelGrid<pcl::PointXYZI>
//     filter; filter.setInputCloud(cloud); filter.setLeafSize(0.1f, 0.1f,
//     0.1f); filter.filter(*filtered_cloud);

//     EXPECT_LT(filtered_cloud->points.size(), cloud->points.size()); //
//     уменьшениe точек
// }

// TEST(SystemStressTest, HighFrequencyScanning) {
//     auto node = rclcpp::Node::make_shared("test_node");
//     Lidar lidar(node);
//     lidar.configure(1.0, 100, -45.0, 45.0, 50.0, M_PI / 360.0);

//     for (int i = 0; i < 1000; ++i) {
//         auto cloud = lidar.scan();
//         ASSERT_GT(cloud->points.size(), 0); // oблако точек генерируется
//     }
// }

// TEST(StressTest, ComplexObjectDynamics) {
//     auto node = rclcpp::Node::make_shared("test_node");
//     Lidar lidar(node);
//     lidar.configure(1.0, 50, -45.0, 45.0, 50.0, M_PI / 180.0);

//     std::vector<std::shared_ptr<Object>> objects;
//     for (int i = 0; i < 500; ++i) {
//         auto sphere = std::make_shared<Sphere>(
//             Position3D(i * 0.5, i * 0.5, 0.0, 0.0, 0.0, 0.0), 0.5);
//         lidar.addObject(sphere);
//         objects.push_back(sphere);
//     }

//     for (int step = 0; step < 100; ++step) {
//         for (int i = 0; i < objects.size(); ++i) {
//             objects[i]->setPosition(Position3D(i * 0.5, step * 0.1, 0.0, 0.0,
//             0.0, 0.0));
//         }
//         auto cloud = lidar.scan();
//         EXPECT_GT(cloud->points.size(), 0);
//     }
// }

// TEST(StressTest, LongTermStability) {
//     auto node = rclcpp::Node::make_shared("test_node");
//     Lidar lidar(node);
//     lidar.configure(1.0, 30, -45.0, 45.0, 20.0, M_PI / 180.0);

//     for (int i = 0; i < 100000; ++i) {
//         auto cloud = lidar.scan();
//         ASSERT_GT(cloud->points.size(), 0);
//     }
// }
