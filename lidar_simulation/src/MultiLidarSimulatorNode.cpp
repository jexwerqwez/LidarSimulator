#include "MultiLidarSimulatorNode.h"
using namespace std::chrono_literals;

namespace multi_lidar_sim
{

MultiLidarSimulatorNode::MultiLidarSimulatorNode(const rclcpp::NodeOptions & options)
: Node("multi_lidar_simulator", options)
{
  // 1) читаем конфиг
  std::string cfg_file;
  this->declare_parameter("config_file", "config/multi_lidar_simulation.yaml");
  this->get_parameter("config_file", cfg_file);
  loadConfig(cfg_file);

  // 2) создаём сцену
  // {
  //   Position3D p{0,0,0, 1,0,0};
  //   objects_.push_back(std::make_shared<Plane>(p, 10.0, 10.0));
  // }
  objects_.push_back(std::make_shared<Sphere>(Position3D{3.0, 0.0, 1.0, 0.0, 0.0, 0.0}, 1.0));
  objects_.push_back(std::make_shared<Sphere>(Position3D{-2.0, 2.0, 0.5, 0.0, 0.0, 0.0}, 0.8));
  objects_.push_back(std::make_shared<Sphere>(Position3D{-3.0, -1.0, -1.0, 0.0, 0.0, 0.0}, 0.2));
  objects_.push_back(std::make_shared<Cylinder>(Position3D{2,2,0, 0,0,0}, 1.0, 1.0));

  // 3) сразу отрисуем сцену и положение лидаров
  visualization_ = std::make_unique<Visualization>(this);
  visualization_->publishSceneMarkers(objects_);

  // 4) Создаём каждый Lidar через единый конструктор
  for (size_t i = 0; i < configs_.size(); ++i) {
    auto &cfg = configs_[i];
    // Новый конструктор Lidar
    auto lidar = std::make_unique<Lidar>(
      this,    // node
      cfg.name,              // name
      cfg.name + "_frame",   // frame_id
      cfg.topic,             // topic
      cfg.height,            // lidar_height
      cfg.num_lasers,        // num_lasers
      cfg.alpha_begin,       // alpha_begin (deg)
      cfg.alpha_end,         // alpha_end   (deg)
      cfg.laser_range,       // laser_range
      cfg.horizontal_step    // horizontal_step (rad)
    );
    // Задаём позицию в мировых координатах
    lidar->setPosition(cfg.pose);
    // Сохраняем
    lidars_.push_back(std::move(lidar));
    pubs_.push_back(
      create_publisher<sensor_msgs::msg::PointCloud2>(
        cfg.topic,
        rclcpp::QoS(10).transient_local().reliable()
      )
    );
    // И сразу отрисовываем маркер самого лидара
    visualization_->publishLidarPose(cfg.pose, cfg.name, static_cast<int>(i));
  }

  // 5) запускаем таймер на публикацию облаков
  timer_ = this->create_wall_timer(100ms, std::bind(&MultiLidarSimulatorNode::onTimer, this));
}

void MultiLidarSimulatorNode::onTimer()
{
  // для каждого лидара
  for (size_t i = 0; i < lidars_.size(); ++i) {
    auto [cloud, noisy] = lidars_[i]->scan(objects_);  // прокидываем ВСЕ объекты сцены

    // публикуем чистое облако (или noisy, как вам нужно)
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*noisy, msg);
    msg.header.stamp    = now();
    // msg.header.frame_id = lidars_[i]->getFrameId();
    msg.header.frame_id = "map";
    pubs_[i]->publish(msg);

    lidars_[i]->publishTransform();  // TF трансформ той же частотой
  }
}


void MultiLidarSimulatorNode::loadConfig(const std::string & yaml_path)
{
  YAML::Node root = YAML::LoadFile(yaml_path);
  for (auto n : root["lidars"]) {
    LidarConfig cfg;
    cfg.name      = n["name"].as<std::string>();
    cfg.topic     = n["topic"].as<std::string>();
    auto p        = n["pose"];
    cfg.pose.position.x = p["x"].as<double>();
    cfg.pose.position.y = p["y"].as<double>();
    cfg.pose.position.z = p["z"].as<double>();
    // roll/pitch/yaw в radians:
    cfg.pose.orientation =
      Eigen::AngleAxisd(p["yaw"].as<double>(),   Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(p["pitch"].as<double>(), Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(p["roll"].as<double>(),  Eigen::Vector3d::UnitX());
    cfg.height         = n["height"].as<double>();
    cfg.num_lasers     = n["num_lasers"].as<int>();
    cfg.alpha_begin    = n["alpha_begin"].as<double>();
    cfg.alpha_end      = n["alpha_end"].as<double>();
    cfg.laser_range    = n["laser_range"].as<double>();
    cfg.horizontal_step= n["horizontal_step"].as<double>();
    configs_.push_back(cfg);
  }
}

}  // namespace multi_lidar_sim
