#include "cloud_merger/cloud_merger_node.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std::chrono_literals;

namespace cloud_merger
{

CloudMergerNode::CloudMergerNode(const rclcpp::NodeOptions& options)
: Node("cloud_merger_node", options)
{
  // --- читаем параметры ---
  this->declare_parameter<std::vector<std::string>>("input_topics", std::vector<std::string>{});

  this->declare_parameter<std::string>             ("output_topic", "/merged_cloud");
  this->declare_parameter<double>                  ("publish_rate", 1.0);
  this->declare_parameter<double>                  ("voxel_leaf_size", 0.1);
  this->declare_parameter<double>                  ("min_scale", 0.01);
  this->declare_parameter<int>                     ("n_octaves", 3);
  this->declare_parameter<int>                     ("n_scales_per_octave", 4);
  this->declare_parameter<double>                  ("min_contrast", 0.001);

  this->get_parameter("input_topics",       input_topics_);
  this->get_parameter("output_topic",       output_topic_);
  this->get_parameter("publish_rate",       publish_rate_);
  this->get_parameter("voxel_leaf_size",    voxel_leaf_size_);
  this->get_parameter("min_scale",          min_scale_);
  this->get_parameter("n_octaves",          n_octaves_);
  this->get_parameter("n_scales_per_octave",n_scales_per_octave_);
  this->get_parameter("min_contrast",       min_contrast_);

  // резервируем места
  const size_t N = input_topics_.size();
  clouds_.assign(N, nullptr);
  received_.assign(N, false);

  // создаём подписчики
  subs_.reserve(N);
  for (size_t i = 0; i < N; ++i) {
    subs_.push_back(
      this->create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topics_[i],
        rclcpp::QoS(10),
        [this, i](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          cloudCallback(i, msg);
        }
      )
    );
  }

  // паблишер склеенного облака
  merged_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    output_topic_, rclcpp::QoS(10).transient_local().reliable()
  );

  // таймер публикации
  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(1.0/publish_rate_),
    std::bind(&CloudMergerNode::publishMergedCloud, this)
  );
}

PointCloudPtr CloudMergerNode::removeGroundPlane(const PointCloudPtr &cloud)
{
  pcl::SACSegmentation<PointT> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.07);  // допуск в метрах до плоскости

  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coefficients);

  if (inliers->indices.empty()) {
    RCLCPP_WARN(get_logger(), "Ground plane not found");
    return cloud;
  }

  // Удаляем точки, принадлежащие плоскости
  pcl::ExtractIndices<PointT> extract;
  PointCloudPtr filtered(new PointCloudT());
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.setNegative(true);  // оставить всё КРОМЕ пола
  extract.filter(*filtered);

  return filtered;
}


void CloudMergerNode::cloudCallback(
  size_t idx,
  sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // преобразуем ROS → PCL
  PointCloudT::Ptr pc(new PointCloudT());
  pcl::fromROSMsg(*msg, *pc);
  RCLCPP_INFO(this->get_logger(), "cloud[%zu] received with %lu points. Sample intensity: %.2f",
            idx, pc->size(), pc->points[0].intensity);
  clouds_[idx]  = pc;
  received_[idx] = true;
}

void CloudMergerNode::publishMergedCloud()
{
  // проверим, что у нас есть хотя бы по одному сообщению из всех подписок
  for (bool r : received_) {
    if (!r) {
      RCLCPP_DEBUG(get_logger(), "waiting for all %zu clouds", received_.size());
      return;
    }
  }

  // склеиваем
  auto merged = mergeAll(clouds_);
  if (!merged || merged->empty()) {
    RCLCPP_WARN(get_logger(), "merged cloud is empty");
    return;
  }

  // публикуем
  sensor_msgs::msg::PointCloud2 out;
  pcl::toROSMsg(*merged, out);
  out.header.stamp    = now();
  out.header.frame_id = "map";
  merged_pub_->publish(out);
}

PointCloudT::Ptr CloudMergerNode::mergeAll(const std::vector<PointCloudPtr> &clouds)
{
  // входных облаков нет - возвращаем пустое
  if (clouds.empty()) {
    return PointCloudPtr(new PointCloudT());
  }

  // 1) воксельная фильтрация ко всем облакам
  std::vector<PointCloudPtr> filtered_clouds;
  filtered_clouds.reserve(clouds.size());

  for (auto &c : clouds) {
    if (c && !c->empty()) {
      auto filtered = applyVoxelFilter(c);
      filtered = removeGroundPlane(filtered);  // Удаляем пол
      filtered_clouds.push_back(filtered);
    }
  }
  if (filtered_clouds.empty()) {
    return PointCloudPtr(new PointCloudT());
  }

  // начинаем с копии первого
  PointCloudPtr merged(new PointCloudT(*filtered_clouds[0]));

  // итеративно выравниваем и склеиваем
  // for (size_t i = 1; i < filtered.size(); ++i) {
  //   merged = alignClouds(merged, filtered[i]);
  // }

  // повторно прогоняем через фильтр
  // merged = applyVoxelFilter(merged);

  return merged;
}


PointCloudPtr CloudMergerNode::applyVoxelFilter(const PointCloudPtr &cloud)
{
  pcl::VoxelGrid<PointT> vg;
  PointCloudT::Ptr out(new PointCloudT());
  vg.setInputCloud(cloud);
  vg.setLeafSize(voxel_leaf_size_,voxel_leaf_size_,voxel_leaf_size_);
  vg.filter(*out);
  return out;
}

PointCloudPtr CloudMergerNode::extractKeypoints(const PointCloudPtr &cloud)
{
  pcl::SIFTKeypoint<PointT, pcl::PointWithScale> sift;
  pcl::PointCloud<pcl::PointWithScale> tmp;
  KdTreePtr tree(new pcl::search::KdTree<PointT>());
  sift.setSearchMethod(tree);
  sift.setScales(min_scale_, n_octaves_, n_scales_per_octave_);
  sift.setMinimumContrast(min_contrast_);
  sift.setInputCloud(cloud);
  sift.compute(tmp);

  PointCloudPtr keypoints(new PointCloudT());
  pcl::copyPointCloud(tmp, *keypoints);
  return keypoints;
}

DescriptorPtr CloudMergerNode::computeDescriptors(
  const PointCloudPtr &keypoints,
  const PointCloudPtr &surface)
{
  // 1) вычисляем нормали
  pcl::NormalEstimation<PointT, pcl::Normal> ne;
  ne.setInputCloud(surface);
  KdTreePtr tree(new pcl::search::KdTree<PointT>());
  ne.setSearchMethod(tree);
  ne.setRadiusSearch(0.1);
  auto normals = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>());
  ne.compute(*normals);

  // 2) FPFH
  pcl::FPFHEstimation<PointT, pcl::Normal, DescriptorT> fpfh;
  fpfh.setInputCloud(keypoints);
  fpfh.setInputNormals(normals);
  fpfh.setSearchSurface(surface);
  fpfh.setSearchMethod(tree);
  fpfh.setRadiusSearch(0.2);

  DescriptorPtr descriptors(new DescriptorCloudT());
  fpfh.compute(*descriptors);
  return descriptors;
}

PointCloudPtr CloudMergerNode::alignClouds(
  const PointCloudPtr &source,
  const PointCloudPtr &target)
{
  // A) ключевые точки + дескрипторы «на лету»
  auto sk = extractKeypoints(source);
  auto tk = extractKeypoints(target);
  auto sd = computeDescriptors(sk, source);
  auto td = computeDescriptors(tk, target);

  // B) RANSAC-выравнивание
  pcl::SampleConsensusPrerejective<PointT, PointT, DescriptorT> align;
  align.setInputSource(sk);
  align.setSourceFeatures(sd);
  align.setInputTarget(tk);
  align.setTargetFeatures(td);
  align.setMaximumIterations(50000);
  align.setNumberOfSamples(3);
  align.setCorrespondenceRandomness(5);
  align.setSimilarityThreshold(0.9f);
  align.setMaxCorrespondenceDistance(2.5f * voxel_leaf_size_);
  align.setInlierFraction(0.25f);

  PointCloudPtr aligned(new PointCloudT());
  align.align(*aligned);
  if (!align.hasConverged()) {
    RCLCPP_WARN(get_logger(), "pairwise alignment failed");
    return source;
  }

  // C) трансформируем исходное облако и склеиваем
  pcl::PointCloud<PointT> tmp;
  pcl::transformPointCloud(*source, tmp, align.getFinalTransformation());

  PointCloudPtr merged(new PointCloudT());
  *merged = tmp;
  *merged += *target;
  return merged;
}

}  // namespace cloud_merger