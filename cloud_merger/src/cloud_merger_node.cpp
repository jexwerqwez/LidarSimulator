#include "cloud_merger/cloud_merger_node.h"

#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <chrono>
namespace fs = std::filesystem;
using namespace std::chrono_literals;

namespace cloud_merger
{

CloudMergerNode::CloudMergerNode(const rclcpp::NodeOptions & options)
: Node("cloud_merger_node", options)
{
  this->declare_parameter<std::string>("input_file_1", "/tmp/lidar_clouds/cloud1.pcd");
  this->declare_parameter<std::string>("input_file_2", "/tmp/lidar_clouds/cloud2.pcd");
  this->declare_parameter<std::string>("output_file", "/tmp/lidar_clouds/merged_cloud.pcd");
  this->declare_parameter<std::string>("output_topic", "/merged_cloud");
  this->declare_parameter("voxel_leaf_size", 0.1);
  this->declare_parameter("min_scale", 0.01);
  this->declare_parameter("n_octaves", 3);
  this->declare_parameter("n_scales_per_octave", 4);
  this->declare_parameter("min_contrast", 0.001);

  this->get_parameter("input_file_1", input_file_1_);
  this->get_parameter("input_file_2", input_file_2_);
  this->get_parameter("output_file", output_file_);
  this->get_parameter("output_topic", output_topic_);
  this->get_parameter("voxel_leaf_size", voxel_leaf_size_);
  this->get_parameter("min_scale", min_scale_);
  this->get_parameter("n_octaves", n_octaves_);
  this->get_parameter("n_scales_per_octave", n_scales_per_octave_);
  this->get_parameter("min_contrast", min_contrast_);

  merged_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    output_topic_,
    rclcpp::QoS(10).transient_local().reliable()
  );

  tryMerge();

  timer_ = this->create_wall_timer(
    500ms, std::bind(&CloudMergerNode::publishMergedCloud, this));
}

void CloudMergerNode::tryMerge()
{
  PointCloudT::Ptr cloud1(new PointCloudT);
  PointCloudT::Ptr cloud2(new PointCloudT);

  if (pcl::io::loadPCDFile<PointT>(input_file_1_, *cloud1) == -1) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load %s", input_file_1_.c_str());
    return;
  }
  if (pcl::io::loadPCDFile<PointT>(input_file_2_, *cloud2) == -1) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load %s", input_file_2_.c_str());
    return;
  }

  mergeClouds(cloud1, cloud2);
}

void CloudMergerNode::mergeClouds(const PointCloudT::Ptr & cloud1, const PointCloudT::Ptr & cloud2)
{
  auto filtered1 = applyVoxelFilter(cloud1);
  auto filtered2 = applyVoxelFilter(cloud2);

  auto keypoints1 = extractKeypoints(filtered1);
  auto keypoints2 = extractKeypoints(filtered2);

  auto descriptors1 = computeDescriptors(keypoints1, filtered1);
  auto descriptors2 = computeDescriptors(keypoints2, filtered2);

  auto aligned = alignClouds(filtered1, filtered2, descriptors1, descriptors2, keypoints1, keypoints2);

  if (!fs::exists(output_file_)) {
    pcl::io::savePCDFileBinary(output_file_, *aligned);
    RCLCPP_INFO(this->get_logger(), "Saved merged cloud to %s", output_file_.c_str());
  } else {
    RCLCPP_WARN(this->get_logger(), "File already exists: %s", output_file_.c_str());
  }

  merged_cloud_ = aligned;
}

void CloudMergerNode::publishMergedCloud()
{
  if (!merged_cloud_ || merged_cloud_->empty()) return;

  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(*merged_cloud_, msg);
  msg.header.frame_id = "map";
  msg.header.stamp = now();
  merged_cloud_pub_->publish(msg);
}

pcl::PointCloud<PointT>::Ptr CloudMergerNode::applyVoxelFilter(const PointCloudT::Ptr & cloud)
{
  pcl::VoxelGrid<PointT> voxel;
  PointCloudT::Ptr filtered(new PointCloudT());
  voxel.setInputCloud(cloud);
  voxel.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
  voxel.filter(*filtered);
  return filtered;
}

pcl::PointCloud<PointT>::Ptr CloudMergerNode::extractKeypoints(const PointCloudT::Ptr & cloud)
{
  pcl::SIFTKeypoint<PointT, pcl::PointWithScale> sift;
  pcl::PointCloud<pcl::PointWithScale>::Ptr temp(new pcl::PointCloud<pcl::PointWithScale>());
  pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());

  sift.setSearchMethod(tree);
  sift.setScales(min_scale_, n_octaves_, n_scales_per_octave_);
  sift.setMinimumContrast(min_contrast_);
  sift.setInputCloud(cloud);
  sift.compute(*temp);

  PointCloudT::Ptr keypoints(new PointCloudT());
  pcl::copyPointCloud(*temp, *keypoints);
  return keypoints;
}

DescriptorCloudT::Ptr CloudMergerNode::computeDescriptors(const PointCloudT::Ptr & keypoints,
                                                          const PointCloudT::Ptr & surface)
{
  pcl::NormalEstimation<PointT, pcl::Normal> ne;
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
  pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());

  ne.setInputCloud(surface);
  ne.setSearchMethod(tree);
  ne.setRadiusSearch(0.1);
  ne.compute(*normals);

  pcl::FPFHEstimation<PointT, pcl::Normal, DescriptorT> fpfh;
  DescriptorCloudT::Ptr descriptors(new DescriptorCloudT);
  fpfh.setInputCloud(keypoints);
  fpfh.setInputNormals(normals);
  fpfh.setSearchSurface(surface);
  fpfh.setSearchMethod(tree);
  fpfh.setRadiusSearch(0.2);
  fpfh.compute(*descriptors);

  return descriptors;
}

pcl::PointCloud<PointT>::Ptr CloudMergerNode::alignClouds(const PointCloudT::Ptr & source,
                                                          const PointCloudT::Ptr & target,
                                                          const DescriptorCloudT::Ptr & source_desc,
                                                          const DescriptorCloudT::Ptr & target_desc,
                                                          const PointCloudT::Ptr & source_kp,
                                                          const PointCloudT::Ptr & target_kp)
{
  pcl::SampleConsensusPrerejective<PointT, PointT, DescriptorT> align;
  align.setInputSource(source_kp);
  align.setSourceFeatures(source_desc);
  align.setInputTarget(target_kp);
  align.setTargetFeatures(target_desc);
  align.setMaximumIterations(50000);
  align.setNumberOfSamples(3);
  align.setCorrespondenceRandomness(5);
  align.setSimilarityThreshold(0.9f);
  align.setMaxCorrespondenceDistance(2.5f * voxel_leaf_size_);
  align.setInlierFraction(0.25f);

  PointCloudT::Ptr aligned(new PointCloudT());
  align.align(*aligned);

  if (!align.hasConverged()) {
    RCLCPP_WARN(this->get_logger(), "Alignment failed");
    return source;
  }

  PointCloudT::Ptr merged(new PointCloudT());
  pcl::transformPointCloud(*source, *aligned, align.getFinalTransformation());
  *merged = *aligned + *target;
  return merged;
}

}  // namespace cloud_merger
