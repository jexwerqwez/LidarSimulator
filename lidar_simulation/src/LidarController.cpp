#include "../include/LidarController.h"

LidarController::LidarController(rclcpp::Node::SharedPtr node)
    : node_(node), view_(node) {
  // инициализация параметров для лидара и плоскостей
  double lidar_height = node_->declare_parameter(
      "lidar_height", 1.0);  // высота лидара над землёй
  int num_lasers = node_->declare_parameter(
      "num_lasers", 30);  // количество лазеров по вертикали
  double alpha_begin =
      node_->declare_parameter("alpha_begin", -45.0);  // начальный угол
  double alpha_end =
      node_->declare_parameter("alpha_end", 45.0);  // конечный угол
  double laser_range =
      node_->declare_parameter("laser_range", 20.0);  // дальность лазера
  // параметры для плоскостей
  auto planes_tilt_angles = node_->declare_parameter<std::vector<double>>(
      "planes_tilt_angles", {20.0, -20.0});
  auto planes_heights = node_->declare_parameter<std::vector<double>>(
      "planes_heights", {1.0, 1.5});

  // конфигурация модели
  model_.configure(lidar_height, num_lasers, alpha_begin, alpha_end,
                   laser_range, planes_tilt_angles, planes_heights);
  model_.findIntersections();

  // таймер для публикации данных
  timer_ =
      node_->create_wall_timer(std::chrono::milliseconds(100),
                               std::bind(&LidarController::publishData, this));
}

void LidarController::publishData() {
  view_.publishPointCloud(model_.getPointCloud());  // публикация облака точек
  view_.publishPlaneMarkers(model_.getPlanes(),
                            "lidar_frame");  // публикация маркеров
}
