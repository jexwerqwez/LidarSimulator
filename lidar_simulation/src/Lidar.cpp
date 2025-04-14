#include "../include/Lidar.h"

#include <cmath>
#include <random>

#include <rclcpp/parameter.hpp>

Lidar::Lidar(rclcpp::Node::SharedPtr node) : node_(node) {
  position_ = Position3D(0.0, 0.0, 0.0, 1.0, 0.0, 0.0); // Начальная позиция
  loadParametersFromYaml();
  configure(lidar_height_, num_lasers_, alpha_begin_, alpha_end_, laser_range_, horizontal_step_);

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
  cloud_publisher_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("lidar/point_cloud", 10);
}


void Lidar::loadParametersFromYaml() {
  node_->declare_parameter("lidar.lidar_height", 0.0);
  node_->declare_parameter("lidar.num_lasers", 30);
  node_->declare_parameter("lidar.alpha_begin", -45.0);
  node_->declare_parameter("lidar.alpha_end", 45.0);
  node_->declare_parameter("lidar.laser_range", 20.0);
  node_->declare_parameter("lidar.horizontal_step", 0.0174533);

  lidar_height_ = node_->get_parameter("lidar.lidar_height").as_double();
  num_lasers_ = node_->get_parameter("lidar.num_lasers").as_int();
  alpha_begin_ = node_->get_parameter("lidar.alpha_begin").as_double();
  alpha_end_ = node_->get_parameter("lidar.alpha_end").as_double();
  laser_range_ = node_->get_parameter("lidar.laser_range").as_double();
  horizontal_step_ = node_->get_parameter("lidar.horizontal_step").as_double();
}

void Lidar::configure(double lidar_height, int num_lasers, double alpha_begin,
                      double alpha_end, double laser_range,
                      double horizontal_step) {
  lidar_height_ = lidar_height;
  num_lasers_ = num_lasers;
  alpha_begin_ = alpha_begin;
  alpha_end_ = alpha_end;
  laser_range_ = laser_range;
  horizontal_step_ = horizontal_step;

  generateRays();
}

// функция для добавления объектов в лидар
void Lidar::addObject(std::shared_ptr<Object> object) {
  objects_.push_back(object);
}

// функция для генерации лучей с учетом заданных углов
void Lidar::generateRays() {
  rays_.clear();
  // переводим углы в радианы и рассчитываем шаг между лазерами
  double alpha_begin_rad = alpha_begin_ * M_PI / 180.0;
  double alpha_end_rad = alpha_end_ * M_PI / 180.0;
  double vertical_step = (alpha_end_rad - alpha_begin_rad) / (num_lasers_ - 1);
  // циклы для создания всех лучей, проходящих вокруг по горизонтали и с
  // заданными вертикальными углами
  for (double yaw = 0.0; yaw < 2 * M_PI;
       yaw += horizontal_step_) {  // проходим круг по горизонтали с шагом в
                                   // один градус
    for (int i = 0; i < num_lasers_;
         ++i) {  // проходим по всем лазерным вертикальным углам
      double vertical_angle =
          alpha_begin_rad +
          i * vertical_step;  // высчитываем текущий вертикальный угол
      Eigen::Vector3d ray;
      ray.x() = cos(vertical_angle) *
                cos(yaw);  // координата x с учетом вертикального и
                           // горизонтального угла
      ray.y() = cos(vertical_angle) *
                sin(yaw);  // координата y с учетом вертикального и
                           // горизонтального угла
      ray.z() = sin(vertical_angle);  // координата z по вертикальному углу
      rays_.push_back(ray.normalized());
    }
  }
}

// функция для поиска точек пересечения лучей с объектами
std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> Lidar::scan() {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr noisy_cloud(new pcl::PointCloud<pcl::PointXYZI>());

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<double> drop_dist(0.0, 1.0);
  std::normal_distribution<double> noise_dist(0.0, 0.02);
  double drop_ratio = 0.1;  // 10% точек будет удалено

  // для каждого луча ищем ближайшую точку пересечения
  for (const auto &ray : rays_) {
    if (drop_dist(gen) < drop_ratio) {
        continue;  // пропускаем точку
    }

    Eigen::Vector3d ray_origin(0.0, 0.0, lidar_height_);
    Eigen::Vector3d ray_direction = ray;

    Point3D nearest_point;
    double min_distance = laser_range_;
    bool hit = false;

    for (const auto &object : objects_) {
      Point3D intersection;
      if (object->intersects(ray_origin, ray_direction, laser_range_,
                             intersection)) {
        double distance =
            std::sqrt(std::pow(intersection.x - ray_origin.x(), 2) +
                      std::pow(intersection.y - ray_origin.y(), 2) +
                      std::pow(intersection.z - ray_origin.z(), 2));
        if (distance < min_distance) {
          min_distance = distance;
          nearest_point = intersection;
          hit = true;
        }
      }
    }

    // сохраняем ближайшую точку в облако точек
    if (hit) {
      pcl::PointXYZI point;
      point.x = nearest_point.x;
      point.y = nearest_point.y;
      point.z = nearest_point.z;
      point.intensity = min_distance;
      cloud->points.push_back(point);

      // добавляем шум
      double noise_factor = 0.01 * min_distance + 0.005 * min_distance * min_distance;  // квадратично
      std::normal_distribution<double> noise_dist(0.0, noise_factor);

      pcl::PointXYZI noisy_point = point;
      noisy_point.x += noise_dist(gen);
      noisy_point.y += noise_dist(gen);
      noisy_point.z += noise_dist(gen);
      noisy_cloud->points.push_back(noisy_point);
    }
  }

  // настраиваем параметры облака точек
  cloud->width = static_cast<uint32_t>(cloud->points.size());
  cloud->height = 1;
  cloud->is_dense = false;

  noisy_cloud->width = noisy_cloud->points.size();
  noisy_cloud->height = 1;
  noisy_cloud->is_dense = false;

  // Возвращаем два облака
  return std::make_pair(cloud, noisy_cloud);
}

// std::vector<Eigen::Vector3d> Lidar::getRays() const { return rays_; }

void Lidar::publishPointCloud() {
  auto [cloud, noisy_cloud] = scan();
  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(*cloud, cloud_msg);
  cloud_msg.header.frame_id = "lidar_frame";
  cloud_msg.header.stamp = node_->now();
  cloud_publisher_->publish(cloud_msg);
}

Eigen::Vector3d Lidar::getPosition() const {
  return {position_.position.x, position_.position.y, position_.position.z};
}

void Lidar::updatePosition(double linear_velocity, double angular_velocity, double dt) {
  Eigen::Vector3d forward = position_.orientation * Eigen::Vector3d(1, 0, 0);

  position_.position.x += linear_velocity * dt * forward.x();
  position_.position.y += linear_velocity * dt * forward.y();
  position_.position.z += linear_velocity * dt * forward.z();

  Eigen::AngleAxisd yawRotation(angular_velocity * dt, Eigen::Vector3d::UnitZ());
  position_.orientation = position_.orientation * yawRotation;

  RCLCPP_INFO(node_->get_logger(), "Lidar position updated: x=%.2f, y=%.2f, yaw=%.2f",
              position_.position.x, position_.position.y, angular_velocity * dt);

  publishTransform();
  publishPointCloud();
}

void Lidar::publishTransform() {
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = node_->now();
  transform.header.frame_id = "map";  // Глобальная система координат
  transform.child_frame_id = "lidar_frame";

  transform.transform.translation.x = position_.position.x;
  transform.transform.translation.y = position_.position.y;
  transform.transform.translation.z = position_.position.z;

  transform.transform.rotation.x = position_.orientation.x();
  transform.transform.rotation.y = position_.orientation.y();
  transform.transform.rotation.z = position_.orientation.z();
  transform.transform.rotation.w = position_.orientation.w();

  tf_broadcaster_->sendTransform(transform);
}


void Lidar::savePointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {
  const std::string dir = "/tmp/lidar_clouds/";

  // Создаём директорию, если её нет
  if (!std::filesystem::exists(dir)) {
    std::filesystem::create_directories(dir);
  }

  // Получаем текущее системное время
  auto now = std::chrono::system_clock::now();
  std::time_t now_c = std::chrono::system_clock::to_time_t(now);

  // Форматируем дату и время в строку
  std::ostringstream oss;
  oss << std::put_time(std::localtime(&now_c), "cloud_%Y-%m-%d_%H-%M-%S.pcd");
  std::string filename = dir + oss.str();

  // Сохраняем файл
  if (pcl::io::savePCDFileBinary(filename, *cloud) == 0) {
    RCLCPP_INFO(node_->get_logger(), "Saved point cloud to %s", filename.c_str());
  } else {
    RCLCPP_ERROR(node_->get_logger(), "Failed to save point cloud to %s", filename.c_str());
  }
}
