#include "trunk_detector/trunk_detector.h"

#include <pcl_conversions/pcl_conversions.h>

#include <cmath>
#include <unordered_map>

TrunkDetector::TrunkDetector(const rclcpp::NodeOptions& options)
    : Node("trunk_detector_node", options),
      cloud_(new pcl::PointCloud<PointT>) {
  this->declare_parameter("input_topic", "/lidar/point_cloud");
  this->declare_parameter("output_topic", "detector_output");
  this->declare_parameter("grid_size", 0.4);
  this->declare_parameter("height_threshold", 0.3);
  this->declare_parameter("min_intensity", 0.05);
  this->declare_parameter("max_distance", 40.0);

  grid_size_ = this->get_parameter("grid_size").as_double();
  height_threshold_ = this->get_parameter("height_threshold").as_double();
  min_intensity_ = this->get_parameter("min_intensity").as_double();
  max_distance_ = this->get_parameter("max_distance").as_double();

  std::string input_topic = this->get_parameter("input_topic").as_string();
  std::string output_topic = this->get_parameter("output_topic").as_string();

  sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic, rclcpp::SensorDataQoS(),
      std::bind(&TrunkDetector::pointCloudCallback, this,
                std::placeholders::_1));

  pub_ = this->create_publisher<trunk_detector_msgs::msg::TrunkPoseArray>(
      output_topic, 10);
}

TrunkDetector::~TrunkDetector() {}

void TrunkDetector::pointCloudCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  pcl::fromROSMsg(*msg, *cloud_);
  trunk_detector_msgs::msg::TrunkPoseArray out;
  out.header = msg->header;

  std::unordered_map<std::pair<int, int>, std::vector<const PointT*>> grid;

  for (const auto& pt : cloud_->points) {
    double dist2 = pt.x * pt.x + pt.y * pt.y;
    if (dist2 > max_distance_ * max_distance_ || pt.intensity < min_intensity_)
      continue;

    int gx = static_cast<int>(std::floor(pt.x / grid_size_));
    int gy = static_cast<int>(std::floor(pt.y / grid_size_));
    grid[{gx, gy}].push_back(&pt);
  }

  for (const auto& cell : grid) {
    const auto& points = cell.second;
    double z_min = 1e6, z_max = -1e6;
    double sum_x = 0, sum_y = 0;
    for (const auto* pt : points) {
      sum_x += pt->x;
      sum_y += pt->y;
      if (pt->z < z_min) z_min = pt->z;
      if (pt->z > z_max) z_max = pt->z;
    }

    if (z_max - z_min < height_threshold_) continue;

    trunk_detector_msgs::msg::TrunkPose tp;
    tp.x = sum_x / points.size();
    tp.y = sum_y / points.size();
    tp.r = std::hypot(grid_size_, grid_size_);
    tp.h = z_max - z_min;
    out.trunks.push_back(tp);
  }
  RCLCPP_INFO(this->get_logger(), "Trunks found: %lu out of %lu points",
              out.trunks.size(), cloud_->size());

  pub_->publish(out);
}
