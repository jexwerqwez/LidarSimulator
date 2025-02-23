#include "../include/Lidar.h"

#include <cmath>
#include <random>

Lidar::Lidar(rclcpp::Node::SharedPtr node)
    : node_(node),
      lidar_height_(1.0),
      num_lasers_(30),
      alpha_begin_(-45.0),
      alpha_end_(45.0),
      laser_range_(20.0),
      horizontal_step_(M_PI / 180.0)  // 1 градус
{}

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
      double noise_stddev = 0.02 * min_distance;  // 2% от расстояния
      std::normal_distribution<double> noise_dist(0.0, noise_stddev);
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

std::vector<Eigen::Vector3d> Lidar::getRays() const { return rays_; }