#pragma once

#include <pcl/ModelCoefficients.h>
#include <pcl/features/fpfh.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <vector>

namespace cloud_merger {

using PointT = pcl::PointXYZI;
using PointCloudT = pcl::PointCloud<PointT>;
using DescriptorT = pcl::FPFHSignature33;
using DescriptorCloudT = pcl::PointCloud<DescriptorT>;

using PointCloudPtr = PointCloudT::Ptr;
using DescriptorPtr = DescriptorCloudT::Ptr;
using KdTreePtr = pcl::search::KdTree<PointT>::Ptr;

class CloudMergerNode : public rclcpp::Node {
 public:
  explicit CloudMergerNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

 private:
  // подписка
  void cloudCallback(size_t idx, sensor_msgs::msg::PointCloud2::SharedPtr msg);
  // публикация
  void publishMergedCloud();
  // алгоритм склейки
  PointCloudPtr mergeAll(const std::vector<PointCloudPtr> &clouds);
  PointCloudPtr applyVoxelFilter(const PointCloudPtr &cloud);
  PointCloudPtr extractKeypoints(const PointCloudPtr &cloud);
  DescriptorPtr computeDescriptors(const PointCloudPtr &keypoints,
                                   const PointCloudPtr &surface);
  PointCloudPtr alignClouds(const PointCloudPtr &source,
                            const PointCloudPtr &target);
  PointCloudPtr removeGroundPlane(const PointCloudPtr &cloud);

  // параметры
  std::vector<std::string> input_topics_;
  std::string output_topic_;
  double publish_rate_;

  double voxel_leaf_size_;
  double min_scale_;
  int n_octaves_;
  int n_scales_per_octave_;
  double min_contrast_;

  // подписчики и хранение входных облаков
  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr>
      subs_;
  std::vector<PointCloudPtr> clouds_;
  std::vector<bool> received_;

  // паблишер и таймер
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr merged_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace cloud_merger