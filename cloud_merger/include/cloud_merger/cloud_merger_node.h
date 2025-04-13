#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/fpfh.h>

#include <string>

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
  void processDirectory();
  pcl::PointCloud<PointT>::Ptr applyVoxelFilter(const pcl::PointCloud<PointT>::Ptr & cloud);
  pcl::PointCloud<PointT>::Ptr extractKeypoints(const pcl::PointCloud<PointT>::Ptr & cloud);
  DescriptorCloudT::Ptr computeDescriptors(const pcl::PointCloud<PointT>::Ptr & keypoints,
                                           const pcl::PointCloud<PointT>::Ptr & surface);
  pcl::PointCloud<PointT>::Ptr alignClouds(const pcl::PointCloud<PointT>::Ptr & source,
                                           const pcl::PointCloud<PointT>::Ptr & target,
                                           const DescriptorCloudT::Ptr & source_desc,
                                           const DescriptorCloudT::Ptr & target_desc,
                                           const pcl::PointCloud<PointT>::Ptr & source_kp,
                                           const pcl::PointCloud<PointT>::Ptr & target_kp);

  std::string input_directory_;
  std::string output_file_;

  double voxel_leaf_size_;
  double min_scale_;
  int n_octaves_;
  int n_scales_per_octave_;
  double min_contrast_;
};

}  // namespace cloud_merger
