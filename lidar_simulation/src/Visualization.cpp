#include "Visualization.h"
#include <visualization_msgs/msg/marker_array.hpp>

Visualization::Visualization(rclcpp::Node * node,
                             const std::string & marker_topic)
: node_(node)
{
  marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
    marker_topic, 10);  
}

void Visualization::publishSceneMarkers(
  const std::vector<std::shared_ptr<Object>> & objects)
{
  visualization_msgs::msg::MarkerArray marker_array;
  int id = 0;
  for (auto & obj : objects) {
    auto m = obj->getMarker(id++);
    m.header.stamp = node_->now();
    m.header.frame_id = "map";
    marker_array.markers.push_back(m);
  }
  marker_pub_->publish(marker_array);
}

void Visualization::publishLidarPose(
  const Position3D & pose,
  const std::string & ns,
  int id)
{
  visualization_msgs::msg::MarkerArray marker_array;

  visualization_msgs::msg::Marker m;
  m.header.stamp = node_->now();
  m.header.frame_id = "map";
  m.ns = ns;
  m.id = id;
  m.type = visualization_msgs::msg::Marker::SPHERE;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.pose.position.x = pose.position.x;
  m.pose.position.y = pose.position.y;
  m.pose.position.z = pose.position.z;
  m.pose.orientation.x = pose.orientation.x();
  m.pose.orientation.y = pose.orientation.y();
  m.pose.orientation.z = pose.orientation.z();
  m.pose.orientation.w = pose.orientation.w();
  m.scale.x = m.scale.y = m.scale.z = 0.2;
  m.color.r = 1.0; m.color.g = 0.0; m.color.b = 0.0; m.color.a = 1.0;

  marker_array.markers.push_back(m);
  marker_pub_->publish(marker_array);
}
