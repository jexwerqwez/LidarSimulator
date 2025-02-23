#include "../include/LidarController.h"

#include "../include/Objects/Plane.h"
#include "../include/Objects/Sphere.h"

LidarController::LidarController(rclcpp::Node::SharedPtr node)
    : lidar_(node), visualization_(node), node_(node) {
  // инициализация параметров для лидара
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
  double horizontal_step_deg = node_->declare_parameter(
      "horizontal_step_deg", 1.0);  // шаг лидара по горизонтали
  double horizontal_step = horizontal_step_deg * M_PI / 180.0;

  lidar_.configure(lidar_height, num_lasers, alpha_begin, alpha_end,
                   laser_range, horizontal_step);

  // инициализация плоскостей
//   Position3D plane1_pos(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
//   auto plane1 = std::make_shared<Plane>(plane1_pos, 10.0, 10.0);
//   objects_.push_back(plane1);
//   lidar_.addObject(plane1);

//   Position3D plane2_pos(1.0, 0.0, 0.0, 10.0 * M_PI / 180.0, 0.0, 0.0);
//   auto plane2 = std::make_shared<Plane>(plane2_pos, 10.0, 10.0);
//   objects_.push_back(plane2);
//   lidar_.addObject(plane2);

  // инициализация сфер
  auto sphere1 =
      std::make_shared<Sphere>(Position3D(3.0, 0.0, 1.0, 0.0, 0.0, 0.0), 1.0);
  auto sphere2 =
      std::make_shared<Sphere>(Position3D(-2.0, 2.0, 0.5, 0.0, 0.0, 0.0), 0.8);
  auto sphere3 = std::make_shared<Sphere>(
      Position3D(-3.0, -1.0, -1.0, 0.0, 0.0, 0.0), 0.2);
  auto sphere4 =
      std::make_shared<Sphere>(Position3D(3.0, -1.0, -1.0, 0.0, 0.0, 0.0), 0.5);
  auto sphere5 =
      std::make_shared<Sphere>(Position3D(-1.0, -3.0, -3.0, 0.0, 1.0, 0.0), 6.0);
  objects_.push_back(sphere1);
  objects_.push_back(sphere2);
  objects_.push_back(sphere3);
  objects_.push_back(sphere4);
  objects_.push_back(sphere5);
  lidar_.addObject(sphere1);
  lidar_.addObject(sphere2);
  lidar_.addObject(sphere3);
  lidar_.addObject(sphere4);
  lidar_.addObject(sphere5);

  // таймер для публикации данных
  timer_ =
      node_->create_wall_timer(std::chrono::milliseconds(100),
                               std::bind(&LidarController::publishData, this));
}

void LidarController::run() { rclcpp::spin(node_); }

// void LidarController::publishData() {
//   auto cloud = lidar_.scan();
//   visualization_.publishPointCloud(cloud);  // публикация облака точек
//   visualization_.publishMarkers(objects_);  // публикация маркеров
// }

void LidarController::publishData() {
    auto cloud_pair = lidar_.scan();
    visualization_.publishPointCloud(cloud_pair.first, cloud_pair.second);
    visualization_.publishMarkers(objects_);
  Eigen::Vector3d ray_origin(0.0, 0.0, 5.0);
  Eigen::Vector3d ray_direction(0.0, 0.0, -1.0);
  double max_range = 10.0;

  Eigen::Vector3d ray_end = ray_origin + ray_direction * max_range;

  visualization_.publishRay(ray_origin, ray_end, "test_ray", 0.05,
                            {1.0, 0.0, 0.0, 1.0});

  Position3D plane_pos(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
  Plane test_plane(plane_pos, 10.0, 10.0);
  visualization_.publishPlane(test_plane, "test_plane", {0.0, 1.0, 0.0, 0.5});
}
