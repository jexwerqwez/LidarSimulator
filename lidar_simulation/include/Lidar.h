#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "Objects/Object.h"
#include "Position3D.h"

class Lidar
{
public:
  Lidar(rclcpp::Node * node,
        const std::string & name,
        const std::string & frame_id,
        const std::string & topic,
        double lidar_height,
        int num_lasers,
        double alpha_begin,
        double alpha_end,
        double laser_range,
        double horizontal_step);

  /// scan(scene_objects) возвращает пару: <идеальное облако, зашумленное облако>
  std::pair<
    pcl::PointCloud<pcl::PointXYZI>::Ptr,
    pcl::PointCloud<pcl::PointXYZI>::Ptr
  >
  scan(const std::vector<std::shared_ptr<Object>> & scene_objects);

  /// Публикует только TF-позу лидара
  void publishTransform();

  std::string getFrameId() const { return frame_id_; }

  /// Установить в мире глобальную позу лидара
  void setPosition(const Position3D & pose);

private:
  void generateRays();

  Position3D position_;
  std::string name_;
  std::string frame_id_;

  double lidar_height_;
  int num_lasers_;
  double alpha_begin_, alpha_end_, laser_range_, horizontal_step_;

  std::vector<Eigen::Vector3d> rays_;

  rclcpp::Node * node_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};
