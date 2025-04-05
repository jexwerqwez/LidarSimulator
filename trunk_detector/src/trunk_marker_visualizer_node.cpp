#include "rclcpp/rclcpp.hpp"
#include "trunk_detector_msgs/msg/trunk_pose_array.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

class TrunkMarkerVisualizer : public rclcpp::Node {
public:
  TrunkMarkerVisualizer() : Node("trunk_marker_visualizer_node") {
    this->declare_parameter("input_topic", "/detector_output");
    this->declare_parameter("output_topic", "/trunk_markers");
    std::string input_topic = this->get_parameter("input_topic").as_string();
    std::string output_topic = this->get_parameter("output_topic").as_string();

    sub_ = this->create_subscription<trunk_detector_msgs::msg::TrunkPoseArray>(
      input_topic, 10,
      std::bind(&TrunkMarkerVisualizer::callback, this, std::placeholders::_1));

    pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(output_topic, 10);
  }

private:
  void callback(const trunk_detector_msgs::msg::TrunkPoseArray::SharedPtr msg) {
    visualization_msgs::msg::MarkerArray marker_array;

    int id = 0;
    for (const auto& trunk : msg->trunks) {
      visualization_msgs::msg::Marker marker;
      marker.header = msg->header;
      marker.ns = "trunks";
      marker.id = id++;
      marker.type = visualization_msgs::msg::Marker::CYLINDER;
      marker.action = visualization_msgs::msg::Marker::ADD;

      marker.pose.position.x = trunk.x;
      marker.pose.position.y = trunk.y;
      marker.pose.position.z = 0.5;  // Высота центра цилиндра

      marker.pose.orientation.w = 1.0;

      // Минимальный радиус для отображения
      double radius = std::abs(trunk.r);
      if (radius < 0.05) {
        radius = 0.05;
      }

      marker.scale.x = std::abs(2.0 * radius);
      marker.scale.y = std::abs(2.0 * radius);
      marker.scale.z = 1.0;

      // Цвет по категории
      switch (trunk.c) {
        case 0: // none
          marker.color.r = 0.5;
          marker.color.g = 0.5;
          marker.color.b = 0.5;
          break;
        case 1: // small
          marker.color.r = 0.2;
          marker.color.g = 1.0;
          marker.color.b = 0.2;
          break;
        case 2: // medium
          marker.color.r = 1.0;
          marker.color.g = 1.0;
          marker.color.b = 0.2;
          break;
        case 3: // big
          marker.color.r = 1.0;
          marker.color.g = 0.2;
          marker.color.b = 0.2;
          break;
        default:
          marker.color.r = 1.0;
          marker.color.g = 1.0;
          marker.color.b = 1.0;
      }

      marker.color.a = 0.8;
      marker.lifetime = rclcpp::Duration::from_seconds(0.5);

      marker_array.markers.push_back(marker);
    }

    pub_->publish(marker_array);
  }

  rclcpp::Subscription<trunk_detector_msgs::msg::TrunkPoseArray>::SharedPtr sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrunkMarkerVisualizer>());
  rclcpp::shutdown();
  return 0;
}
