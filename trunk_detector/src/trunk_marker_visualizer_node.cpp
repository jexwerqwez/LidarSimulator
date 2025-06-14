#include "rclcpp/rclcpp.hpp"
#include "trunk_detector_msgs/msg/trunk_pose_array.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

class TrunkMarkerVisualizer : public rclcpp::Node {
 public:
  TrunkMarkerVisualizer() : Node("trunk_marker_visualizer_node") {
    this->declare_parameter<std::string>("input_topic", "/detector_output");
    this->declare_parameter<std::string>("output_topic", "/trunk_markers");

    auto input_topic = this->get_parameter("input_topic").as_string();
    auto output_topic = this->get_parameter("output_topic").as_string();

    sub_ = this->create_subscription<trunk_detector_msgs::msg::TrunkPoseArray>(
        input_topic, 10,
        std::bind(&TrunkMarkerVisualizer::callback, this,
                  std::placeholders::_1));

    pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        output_topic, 10);
  }

 private:
  void callback(const trunk_detector_msgs::msg::TrunkPoseArray::SharedPtr msg) {
    visualization_msgs::msg::MarkerArray ma;
    int id = 0;

    for (const auto &t : msg->trunks) {
      visualization_msgs::msg::Marker marker;
      marker.header = msg->header;
      marker.ns = "trunk";
      marker.id = id++;
      marker.type = visualization_msgs::msg::Marker::CYLINDER;
      marker.action = visualization_msgs::msg::Marker::ADD;

      // позиция и ориентация
      marker.pose.position.x = t.x;
      marker.pose.position.y = t.y;
      marker.pose.position.z = 0.5;
      marker.pose.orientation.w = 1.0;

      // размер цилиндра
      double radius = std::max(0.05, std::abs(t.r));
      marker.scale.x = 1.0 * radius;
      marker.scale.y = 1.0 * radius;
      marker.scale.z = 1.0;

      // градация по цвету
      double max_height = 2.0;
      double norm = std::clamp(t.h / max_height, 0.0, 1.0);
      marker.color.r = norm;
      marker.color.g = 1.0 - norm;
      marker.color.b = 0.0;
      marker.color.a = 0.8;

      marker.lifetime = rclcpp::Duration::from_seconds(0.0);
      ma.markers.push_back(marker);
    }

    pub_->publish(ma);
  }

  rclcpp::Subscription<trunk_detector_msgs::msg::TrunkPoseArray>::SharedPtr
      sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrunkMarkerVisualizer>());
  rclcpp::shutdown();
  return 0;
}
