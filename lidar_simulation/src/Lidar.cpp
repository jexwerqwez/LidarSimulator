#include "Lidar.h"
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <random>

Lidar::Lidar(rclcpp::Node * node,
             const std::string & name,
             const std::string & frame_id,
             const std::string & topic,
             double lidar_height,
             int num_lasers,
             double alpha_begin,
             double alpha_end,
             double laser_range,
             double horizontal_step)
: node_(node),
  name_(name),
  frame_id_(frame_id),
  lidar_height_(lidar_height),
  num_lasers_(num_lasers),
  // углы приходят в градусах, храним в радианах
  alpha_begin_(alpha_begin * M_PI/180.0),
  alpha_end_(alpha_end * M_PI/180.0),
  laser_range_(laser_range),
  horizontal_step_(horizontal_step)
{
  pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
    topic,
    rclcpp::QoS(10).transient_local().reliable()
  );
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
  generateRays();
}

void Lidar::setPosition(const Position3D & pose) {
  position_ = pose;
}

void Lidar::generateRays() {
  rays_.clear();
  double vstep = (alpha_end_ - alpha_begin_) / (num_lasers_ - 1);
  for (double yaw = 0.0; yaw < 2*M_PI; yaw += horizontal_step_) {
    for (int i = 0; i < num_lasers_; ++i) {
      double vert = alpha_begin_ + i * vstep;
      Eigen::Vector3d r;
      r.x() = std::cos(vert) * std::cos(yaw);
      r.y() = std::cos(vert) * std::sin(yaw);
      r.z() = std::sin(vert);
      rays_.push_back(r.normalized());
    }
  }
}

auto Lidar::scan(const std::vector<std::shared_ptr<Object>> &scene_objects)
  -> std::pair<
       pcl::PointCloud<pcl::PointXYZI>::Ptr,
       pcl::PointCloud<pcl::PointXYZI>::Ptr
     >
{
  auto cloud  = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  auto noisy  = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  std::mt19937 gen{std::random_device{}()};
  std::uniform_real_distribution<> drop{0.0, 1.0};

  for (auto &lr : rays_) {
    if (drop(gen) < 0.1) continue;

    Eigen::Vector3d origin{position_.position.x,
                           position_.position.y,
                           position_.position.z};
    Eigen::Vector3d dir = position_.orientation * lr;

    Point3D nearest;
    double dmin = laser_range_;
    bool hit = false;
    for (auto &obj : scene_objects) {
      Point3D inter;
      if (obj->intersects(origin, dir, laser_range_, inter)) {
        Eigen::Vector3d p(inter.x, inter.y, inter.z);
        double d = (p - origin).norm();
        if (d < dmin) { dmin = d; nearest = inter; hit = true; }
      }
    }
    if (!hit) continue;

    // Идеальная точка
    pcl::PointXYZI pt;
    pt.x = nearest.x;
    pt.y = nearest.y;
    pt.z = nearest.z;
    pt.intensity = static_cast<float>(dmin);
    cloud->points.push_back(pt);

    // Зашумлённая точка
    std::normal_distribution<> noise_dist{0.0, 0.01 * dmin};
    pcl::PointXYZI p2 = pt;
    p2.x += noise_dist(gen);
    p2.y += noise_dist(gen);
    p2.z += noise_dist(gen);
    noisy->points.push_back(p2);
  }

  cloud->width  = static_cast<uint32_t>(cloud->points.size());
  cloud->height = 1;
  cloud->is_dense = false;

  noisy->width  = static_cast<uint32_t>(noisy->points.size());
  noisy->height = 1;
  noisy->is_dense = false;

  return {cloud, noisy};
}


void Lidar::publishTransform() {
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp    = node_->now();
  t.header.frame_id = "map";
  t.child_frame_id  = frame_id_;
  t.transform.translation.x = position_.position.x;
  t.transform.translation.y = position_.position.y;
  t.transform.translation.z = position_.position.z;
  t.transform.rotation.x    = position_.orientation.x();
  t.transform.rotation.y    = position_.orientation.y();
  t.transform.rotation.z    = position_.orientation.z();
  t.transform.rotation.w    = position_.orientation.w();
  tf_broadcaster_->sendTransform(t);
}
