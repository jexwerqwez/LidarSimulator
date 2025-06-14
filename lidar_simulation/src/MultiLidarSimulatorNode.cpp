#include "MultiLidarSimulatorNode.h"

#include <geometry_msgs/msg/twist.hpp>
using namespace std::chrono_literals;

namespace multi_lidar_sim {

MultiLidarSimulatorNode::MultiLidarSimulatorNode(
    const rclcpp::NodeOptions &options)
    : Node("multi_lidar_simulator", options) {
  // читаем конфиг
  std::string cfg_file;
  this->declare_parameter("config_file", "config/multi_lidar_simulation.yaml");
  this->get_parameter("config_file", cfg_file);
  loadConfig(cfg_file);

  // дорога
  for (int i = -2; i <= 2; ++i) {
    for (int j = -1; j <= 1; ++j) {
      Position3D pos;
      pos.position.x = i * 3.0;
      pos.position.y = j * 3.0;
      pos.position.z = 0.0;
      pos.orientation.setIdentity();

      objects_.push_back(std::make_shared<Plane>(pos, 3.0, 3.0));
    }
  }
  objects_.push_back(
      std::make_shared<Plane>(Position3D{0, -4.5, 0.2, 0, 0, 0}, 10, 4));
  objects_.push_back(
      std::make_shared<Plane>(Position3D{0, 4.5, 0.2, 0, 0, 0}, 10, 4));

  // бордюры
  for (int i = -100; i <= 100; ++i) {
    objects_.push_back(std::make_shared<Cylinder>(
        Position3D{i * 0.1, -2.5, 0.1, 0, 0, 0}, 0.1, 0.2));
    objects_.push_back(std::make_shared<Cylinder>(
        Position3D{i * 0.1, 2.5, 0.1, 0, 0, 0}, 0.1, 0.2));
  }

  // машины
  for (int i = -2; i <= 2; ++i) {
    objects_.push_back(std::make_shared<Cylinder>(
        Position3D{double(i) * 2.0, 1.5, 0, 0, 0, 0}, 0.5, 0.5));
  }

  // пешеходы
  objects_.push_back(std::make_shared<Cylinder>(
      Position3D{-2.0, -4.0, 0.0, 0, 0, 0}, 0.15, 1.7));
  objects_.push_back(std::make_shared<Cylinder>(
      Position3D{0.5, 4.2, 0.0, 0, 0, 0}, 0.15, 1.75));
  objects_.push_back(std::make_shared<Cylinder>(
      Position3D{1.5, -4.5, 0.0, 0, 0, 0}, 0.15, 1.8));

  // здания
  objects_.push_back(std::make_shared<Cylinder>(
      Position3D{-6.0, 0.0, 0.0, 0, 0, 0}, 2.0, 6.0));
  objects_.push_back(
      std::make_shared<Cylinder>(Position3D{6.0, 3.0, 0.0, 0, 0, 0}, 1.5, 5.0));
  objects_.push_back(std::make_shared<Cylinder>(
      Position3D{6.0, -3.0, 0.0, 0, 0, 0}, 1.8, 4.5));

  objects_.push_back(
      std::make_shared<Cylinder>(Position3D{3.0, -3.0, 0, 0, 0, 0}, 0.1, 5.0));
  objects_.push_back(
      std::make_shared<Cylinder>(Position3D{-3.0, -3.0, 0, 0, 0, 0}, 0.1, 5.0));
  objects_.push_back(
      std::make_shared<Cylinder>(Position3D{3.0, 3.0, 0, 0, 0, 0}, 0.1, 5.0));
  objects_.push_back(
      std::make_shared<Cylinder>(Position3D{-3.0, 3.0, 0, 0, 0, 0}, 0.1, 5.0));

  // отрисуем сцену и положение лидаров
  visualization_ = std::make_unique<Visualization>(this, "/scene_objects");
  RCLCPP_INFO(this->get_logger(), "Publishing %zu scene objects",
              objects_.size());
  visualization_->publishSceneMarkers(objects_);

  // создаём каждый лидар
  for (size_t i = 0; i < configs_.size(); ++i) {
    auto &cfg = configs_[i];
    auto lidar = std::make_unique<Lidar>(this, cfg.name, cfg.name + "_frame",
                                         cfg.topic, cfg.height, cfg.num_lasers,
                                         cfg.alpha_begin, cfg.alpha_end,
                                         cfg.laser_range, cfg.horizontal_step);
    // задаём позицию лидара
    lidar->setPosition(cfg.pose);
    // сохраняем
    lidars_.push_back(std::move(lidar));
    pubs_.push_back(create_publisher<sensor_msgs::msg::PointCloud2>(
        cfg.topic, rclcpp::QoS(10).transient_local().reliable()));
    // отрисовываем маркер лидара
    visualization_->publishLidarPose(cfg.pose, cfg.name, static_cast<int>(i));
  }

  lidar_control_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/lidar_control", 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        auto &pos = configs_[0].pose.position;
        pos.x += msg->linear.x;
        pos.y += msg->linear.y;
        lidars_[0]->setPosition(configs_[0].pose);
      });

  // запускаем таймер на публикацию облаков
  timer_ = this->create_wall_timer(
      100ms, std::bind(&MultiLidarSimulatorNode::onTimer, this));
  // запускаем таймер на публикацию маркеров объектов
  marker_timer_ = this->create_wall_timer(
      500ms,
      std::bind(&MultiLidarSimulatorNode::publishSceneMarkersTimer, this));
}

void MultiLidarSimulatorNode::publishSceneMarkersTimer() {
  visualization_->publishSceneMarkers(objects_);
}

void MultiLidarSimulatorNode::onTimer() {
  // для каждого лидара
  for (size_t i = 0; i < lidars_.size(); ++i) {
    auto [cloud, noisy] =
        lidars_[i]->scan(objects_);  // прокидываем все объекты сцены

    // публикуем облако
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*noisy, msg);
    msg.header.stamp = now();
    msg.header.frame_id = "map";
    pubs_[i]->publish(msg);

    lidars_[i]->publishTransform();  // преобразуем
  }
}

void MultiLidarSimulatorNode::loadConfig(const std::string &yaml_path) {
  YAML::Node root = YAML::LoadFile(yaml_path);
  for (auto n : root["lidars"]) {
    LidarConfig cfg;
    cfg.name = n["name"].as<std::string>();
    cfg.topic = n["topic"].as<std::string>();
    auto p = n["pose"];
    cfg.pose.position.x = p["x"].as<double>();
    cfg.pose.position.y = p["y"].as<double>();
    cfg.pose.position.z = p["z"].as<double>();
    cfg.pose.orientation =
        Eigen::AngleAxisd(p["yaw"].as<double>(), Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(p["pitch"].as<double>(), Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(p["roll"].as<double>(), Eigen::Vector3d::UnitX());
    cfg.height = n["height"].as<double>();
    cfg.num_lasers = n["num_lasers"].as<int>();
    cfg.alpha_begin = n["alpha_begin"].as<double>();
    cfg.alpha_end = n["alpha_end"].as<double>();
    cfg.laser_range = n["laser_range"].as<double>();
    cfg.horizontal_step = n["horizontal_step"].as<double>();
    configs_.push_back(cfg);
  }
}

}  // namespace multi_lidar_sim
