#pragma once

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "Objects/Object.h"
#include "Position3D.h"

class Visualization {
 public:
  explicit Visualization(rclcpp::Node* node, const std::string& marker_topic =
                                                 "visualization_marker");

  /// публикуем маркеры для всех объектов в глобальной ск
  void publishSceneMarkers(const std::vector<std::shared_ptr<Object>>& objects);

  /// публикуем маркер положения лидара
  void publishLidarPose(const Position3D& pose, const std::string& ns,
                        int id = 0);

 private:
  rclcpp::Node* node_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      marker_pub_;
};
