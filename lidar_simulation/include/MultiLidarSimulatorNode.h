#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <yaml-cpp/yaml.h>
#include <pcl_conversions/pcl_conversions.h>

#include "Lidar.h"
#include "Objects/Plane.h"
#include "Objects/Sphere.h"
#include "Objects/Cylinder.h"
#include "Visualization.h"

#include <memory>
#include <vector>

namespace multi_lidar_sim
{

struct LidarConfig
{
  std::string name;
  std::string topic;
  Position3D pose;
  double height;
  int num_lasers;
  double alpha_begin, alpha_end, laser_range, horizontal_step;
};

class MultiLidarSimulatorNode : public rclcpp::Node
{
public:
  explicit MultiLidarSimulatorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  private:
  void loadConfig(const std::string & yaml_path);
  void onTimer();
  void publishSceneMarkersTimer();

  // конфиги и объекты сцены
  std::vector<LidarConfig> configs_;
  std::vector<std::shared_ptr<Object>> objects_;

  // сами лидары и издатели их облаков
  std::vector<std::unique_ptr<Lidar>> lidars_;
  std::vector<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> pubs_;

  // визуализация
  std::unique_ptr<Visualization> visualization_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr marker_timer_;
};

}  // namespace multi_lidar_sim
