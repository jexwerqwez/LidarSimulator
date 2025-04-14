#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/fpfh.h>

#include <string>
#include <filesystem>

namespace cloud_merger
{

using PointT = pcl::PointXYZI;
using PointCloudT = pcl::PointCloud<PointT>;
using DescriptorT = pcl::FPFHSignature33;
using DescriptorCloudT = pcl::PointCloud<DescriptorT>;

class CloudMergerNode : public rclcpp::Node
{
public:
  explicit CloudMergerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void tryMerge();
  void mergeClouds(const PointCloudT::Ptr & cloud1, const PointCloudT::Ptr & cloud2);

  pcl::PointCloud<PointT>::Ptr applyVoxelFilter(const PointCloudT::Ptr & cloud);
  pcl::PointCloud<PointT>::Ptr extractKeypoints(const PointCloudT::Ptr & cloud);
  DescriptorCloudT::Ptr computeDescriptors(const PointCloudT::Ptr & keypoints,
                                           const PointCloudT::Ptr & surface);
  pcl::PointCloud<PointT>::Ptr alignClouds(const PointCloudT::Ptr & source,
                                           const PointCloudT::Ptr & target,
                                           const DescriptorCloudT::Ptr & source_desc,
                                           const DescriptorCloudT::Ptr & target_desc,
                                           const PointCloudT::Ptr & source_kp,
                                           const PointCloudT::Ptr & target_kp);

  void publishMergedCloud();

  std::string input_file_1_, input_file_2_, output_file_, output_topic_;
  double voxel_leaf_size_;
  double min_scale_;
  int n_octaves_;
  int n_scales_per_octave_;
  double min_contrast_;

  PointCloudT::Ptr merged_cloud_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr merged_cloud_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace cloud_merger
