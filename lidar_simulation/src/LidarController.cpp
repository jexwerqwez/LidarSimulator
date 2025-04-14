#include "../include/LidarController.h"

#include "../include/Objects/Plane.h"
#include "../include/Objects/Sphere.h"
#include "../include/Objects/Cylinder.h"

LidarController::LidarController(rclcpp::Node::SharedPtr node)
    : lidar_(node), visualization_(node), node_(node), linear_velocity_(0.0), angular_velocity_(0.0) {
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
   cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", 10,
    std::bind(&LidarController::cmdVelCallback, this, std::placeholders::_1));
              
  // инициализация плоскостей
  Position3D plane1_pos(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
  auto plane1 = std::make_shared<Plane>(plane1_pos, 10.0, 10.0);
  objects_.push_back(plane1);
  lidar_.addObject(plane1);

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
  // auto sphere5 =
      // std::make_shared<Sphere>(Position3D(-1.0, -3.0, -3.0, 0.0, 1.0, 0.0), 6.0);
  // objects_.push_back(sphere1);
  // objects_.push_back(sphere2);
  // objects_.push_back(sphere3);
  // objects_.push_back(sphere4);
  // objects_.push_back(sphere5);
  // lidar_.addObject(sphere1);
  // lidar_.addObject(sphere2);
  // lidar_.addObject(sphere3);
  // lidar_.addObject(sphere4);
  // lidar_.addObject(sphere5);

    // инициализация цилиндров
    auto cylinder1 = std::make_shared<Cylinder>(Position3D(2.0, 2.0, 0.0, 0.0, 0.0, 0.0), 1.0, 1.0);
    objects_.push_back(cylinder1);
    lidar_.addObject(cylinder1);

  // Подписка на команду движения
  cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", 10, std::bind(&LidarController::cmdVelCallback, this, std::placeholders::_1));

// Таймер для обновления данных
timer_ = node_->create_wall_timer(
    std::chrono::milliseconds(100), std::bind(&LidarController::publishData, this));
    save_service_ = node_->create_service<std_srvs::srv::Trigger>(
      "/save_lidar_cloud",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        auto [_, noisy_cloud] = lidar_.scan();  // только при ручном вызове
        lidar_.savePointCloud(noisy_cloud);     // ← сохраняем тут
        res->success = true;
        res->message = "Cloud saved via service";
      });
  }

// void LidarController::run() {
//     using namespace std::chrono_literals;
    
//     timer_ = node_->create_wall_timer(100ms, [this]() {
//         double dt = 0.1;

//         // Обновление позиции лидара
//         lidar_.updatePosition(linear_velocity_, angular_velocity_, dt);

//         // Получение текущей позиции лидара
//         Eigen::Vector3d lidar_pos = lidar_.getPosition();

//         // Получение облаков точек
//         auto [cloud, noisy_cloud] = lidar_.scan();

//         // Публикация данных
//         visualization_.publishPointCloud(cloud, noisy_cloud);
//         visualization_.publishRay(lidar_pos, lidar_pos + Eigen::Vector3d(0, 0, 0.1),
//                                   "lidar_position", 0.1, {1.0, 0.0, 0.0, 1.0});
//     });
// }


void LidarController::run() {
    rclcpp::Rate rate(10);  // 10 Гц
    double dt = 0.1;  // Шаг по времени (10 Гц)
  
    while (rclcpp::ok()) {
        Eigen::Vector3d lidar_pos = lidar_.getPosition();
      lidar_.updatePosition(linear_velocity_, angular_velocity_, dt);
      rclcpp::spin_some(node_);
      rate.sleep();
    }
  }
  

// void LidarController::publishData() {
//   auto cloud = lidar_.scan();
//   visualization_.publishPointCloud(cloud);  // публикация облака точек
//   visualization_.publishMarkers(objects_);  // публикация маркеров
// }


void LidarController::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    linear_velocity_ = msg->linear.x;  // Получаем линейную скорость
    angular_velocity_ = msg->angular.z;  // Получаем угловую скорость (вращение вокруг оси Z)
    publishData();
  }
  

  void LidarController::publishData() {
    double dt = 0.1;  // период таймера
    lidar_.updatePosition(linear_velocity_, angular_velocity_, dt);

    auto [cloud, noisy_cloud] = lidar_.scan();  // получаем облака
    visualization_.publishPointCloud(cloud, noisy_cloud);  // публикуем в Visualization
    visualization_.publishMarkers(objects_);  // публикуем маркеры
}


//   void LidarController::publishData() {
//     //   auto cloud = lidar_.scan();
//     // visualization_.publishPointCloud(cloud);  // публикация облака точек
//     // visualization_.publishMarkers(objects_);  // публикация маркеров
//     double dt = 0.1;  // период таймера
//     lidar_.updatePosition(linear_velocity_, angular_velocity_, dt);
//   }