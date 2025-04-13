#include "cloud_merger/cloud_merger_node.h"

#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <filesystem>
#include <chrono>

namespace fs = std::filesystem;

namespace cloud_merger
{

CloudMergerNode::CloudMergerNode(const rclcpp::NodeOptions & options)
: Node("cloud_merger_node", options)
{
  this->declare_parameter<std::string>("input_directory", "pointclouds");
  this->declare_parameter<std::string>("output_file", "merged_cloud.pcd");
  this->declare_parameter("voxel_leaf_size", 0.1);
  this->declare_parameter("min_scale", 0.01);
  this->declare_parameter("n_octaves", 3);
  this->declare_parameter("n_scales_per_octave", 4);
  this->declare_parameter("min_contrast", 0.001);

  this->get_parameter("input_directory", input_directory_);
  this->get_parameter("output_file", output_file_);
  this->get_parameter("voxel_leaf_size", voxel_leaf_size_);
  this->get_parameter("min_scale", min_scale_);
  this->get_parameter("n_octaves", n_octaves_);
  this->get_parameter("n_scales_per_octave", n_scales_per_octave_);
  this->get_parameter("min_contrast", min_contrast_);

  processDirectory();
}

void CloudMergerNode::processDirectory()
{
  pcl::PointCloud<PointT>::Ptr accumulated(new pcl::PointCloud<PointT>());

  std::vector<fs::path> files;
  for (const auto & entry : fs::directory_iterator(input_directory_)) {
    if (entry.path().extension() == ".pcd") {
      files.push_back(entry.path());
    }
  }
  std::sort(files.begin(), files.end());

  for (size_t i = 0; i + 1 < files.size(); ++i) {
    pcl::PointCloud<PointT>::Ptr cloud1(new pcl::PointCloud<PointT>());
    pcl::PointCloud<PointT>::Ptr cloud2(new pcl::PointCloud<PointT>());
    pcl::io::loadPCDFile(files[i].string(), *cloud1);
    pcl::io::loadPCDFile(files[i + 1].string(), *cloud2);

    auto filtered1 = applyVoxelFilter(cloud1);
    auto filtered2 = applyVoxelFilter(cloud2);

    auto keypoints1 = extractKeypoints(filtered1);
    auto keypoints2 = extractKeypoints(filtered2);

    auto descriptors1 = computeDescriptors(keypoints1, filtered1);
    auto descriptors2 = computeDescriptors(keypoints2, filtered2);

    auto aligned = alignClouds(filtered1, filtered2, descriptors1, descriptors2, keypoints1, keypoints2);
    *accumulated += *aligned;
  }

  pcl::io::savePCDFileBinary(output_file_, *accumulated);
  RCLCPP_INFO(this->get_logger(), "Saved merged cloud to %s", output_file_.c_str());
}

pcl::PointCloud<PointT>::Ptr CloudMergerNode::applyVoxelFilter(const pcl::PointCloud<PointT>::Ptr & cloud)
{
  pcl::VoxelGrid<PointT> voxel;
  pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
  voxel.setInputCloud(cloud);
  voxel.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
  voxel.filter(*filtered);
  return filtered;
}

pcl::PointCloud<PointT>::Ptr CloudMergerNode::extractKeypoints(const pcl::PointCloud<PointT>::Ptr & cloud)
{
  pcl::SIFTKeypoint<PointT, pcl::PointWithScale> sift;
  pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints_temp(new pcl::PointCloud<pcl::PointWithScale>());
  pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());

  sift.setSearchMethod(tree);
  sift.setScales(min_scale_, n_octaves_, n_scales_per_octave_);
  sift.setMinimumContrast(min_contrast_);
  sift.setInputCloud(cloud);
  sift.compute(*keypoints_temp);

  pcl::PointCloud<PointT>::Ptr keypoints(new pcl::PointCloud<PointT>());
  pcl::copyPointCloud(*keypoints_temp, *keypoints);
  return keypoints;
}

DescriptorCloudT::Ptr CloudMergerNode::computeDescriptors(const pcl::PointCloud<PointT>::Ptr & keypoints,
                                                          const pcl::PointCloud<PointT>::Ptr & surface)
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

pcl::PointCloud<PointT>::Ptr CloudMergerNode::alignClouds(const pcl::PointCloud<PointT>::Ptr & source,
                                                          const pcl::PointCloud<PointT>::Ptr & target,
                                                          const DescriptorCloudT::Ptr & source_desc,
                                                          const DescriptorCloudT::Ptr & target_desc,
                                                          const pcl::PointCloud<PointT>::Ptr & source_kp,
                                                          const pcl::PointCloud<PointT>::Ptr & target_kp)
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

  pcl::PointCloud<PointT>::Ptr aligned(new pcl::PointCloud<PointT>());
  align.align(*aligned);

  if (!align.hasConverged()) {
    RCLCPP_WARN(this->get_logger(), "Alignment failed");
    return source;
  }

  pcl::PointCloud<PointT>::Ptr merged(new pcl::PointCloud<PointT>());
  pcl::transformPointCloud(*source, *aligned, align.getFinalTransformation());
  *merged = *aligned + *target;
  return merged;
}

}  // namespace cloud_merger

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(cloud_merger::CloudMergerNode)
