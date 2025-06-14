#ifndef TRUNK_DETECTOR_H
#define TRUNK_DETECTOR_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <trunk_detector_msgs/msg/trunk_pose_array.hpp>

namespace std {
template <>
struct hash<std::pair<int, int>> {
  size_t operator()(const std::pair<int, int>& p) const {
    return std::hash<int>()(p.first) ^ (std::hash<int>()(p.second) << 1);
  }
};
}  // namespace std

using PointT = pcl::PointXYZI;

class TrunkDetector : public rclcpp::Node {
 public:
  explicit TrunkDetector(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~TrunkDetector();

 private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<trunk_detector_msgs::msg::TrunkPoseArray>::SharedPtr pub_;

  double grid_size_;
  double height_threshold_;
  double min_intensity_;
  double max_distance_;
  int grid_count_;

  pcl::PointCloud<PointT>::Ptr cloud_;
};

#endif  // TRUNK_DETECTOR_H
