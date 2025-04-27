#pragma once

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include "Objects/Object.h"
#include "Position3D.h"

class Visualization
{
public:
  explicit Visualization(rclcpp::Node * node,
                         const std::string & marker_topic = "visualization_marker");

  /// Публикует маркеры для всех объектов (в глобальной системе «map»)
  void publishSceneMarkers(const std::vector<std::shared_ptr<Object>> & objects);

  /// Публикует небольшой маркер положения ЛиДАРа
  void publishLidarPose(const Position3D & pose,
                        const std::string & ns,
                        int id = 0);

private:
  rclcpp::Node * node_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
};
