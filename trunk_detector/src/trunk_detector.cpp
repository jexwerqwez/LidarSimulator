#include "trunk_detector/trunk_detector.h"

#include "pcl_conversions/pcl_conversions.h"
#include "pcl/filters/filter.h"
#include <cmath>

using namespace std;

// Рекурсивная функция поиска индексов точек
void TrunkDetector::index_search(int index, std::map<int, std::vector<int>> & container,
                                 trunk_detector_msgs::msg::TrunkPose & trunk, std::set<int> & checked)
{
  auto it_m = container.find(index);
  if (it_m != container.end() && checked.find(index) == checked.end()) {
    if (!it_m->second.empty() && index == it_m->second[0]) {
      trunk.r = -1;
    } else {
      double intensity = output_->points[it_m->first].intensity / 2.0;
      if (trunk.r < intensity) {
        trunk.r = intensity;
      }
      for (size_t i = 0; i < it_m->second.size(); i++) {
        index_search(it_m->second[i], container, trunk, checked);
      }
    }
    checked.insert(it_m->first);
  }
}

// Конструктор
TrunkDetector::TrunkDetector(const rclcpp::NodeOptions & options)
: Node("trunk_detector_node", options),
  input_(new pcl::PointCloud<PointT>),
  output_(new pcl::PointCloud<PointT>)
{
  update_time_ = this->now();
  // Чтение параметров
  this->declare_parameter("v_shift", 1.9);
  this->declare_parameter("v_delta", 0.75);
  this->declare_parameter("cluster_tolerance", 0.9);
  this->declare_parameter("cluster_max_size", 200.0);
  this->declare_parameter("cluster_min_size", 1.0);
  this->declare_parameter("max_angle_deg", 40.0);
  this->declare_parameter("slices_number", 4.0);
  this->declare_parameter("empty_radius", 5.0);
  this->declare_parameter("small_trunk_hight", 0.5);
  this->declare_parameter("medium_trunk_hight", 1.5);
  this->declare_parameter("map_size", 90.0);
  this->declare_parameter("cell_size", 0.2);
  this->declare_parameter("update_rate", 10.0);
  this->declare_parameter("input_pointclouds_topic", std::string("/lidar/point_cloud"));
  this->declare_parameter("output_pointclouds_topic", std::string("detector_output"));

  // Получение параметров
  std::map<std::string, double> finder_parameters;
  finder_parameters["v_shift"] = this->get_parameter("v_shift").as_double();
  finder_parameters["v_delta"] = this->get_parameter("v_delta").as_double();
  finder_parameters["cluster_tolerance"] = this->get_parameter("cluster_tolerance").as_double();
  finder_parameters["cluster_max_size"] = this->get_parameter("cluster_max_size").as_double();
  finder_parameters["cluster_min_size"] = this->get_parameter("cluster_min_size").as_double();
  finder_parameters["max_angle_deg"] = this->get_parameter("max_angle_deg").as_double();
  finder_parameters["slices_number"] = this->get_parameter("slices_number").as_double();
  finder_parameters["empty_radius"] = this->get_parameter("empty_radius").as_double();
  finder_parameters["cell_size"] = this->get_parameter("cell_size").as_double() / 2.0;

  // Получение остальных параметров
  small_trunk_hight = this->get_parameter("small_trunk_hight").as_double();
  medium_trunk_hight = this->get_parameter("medium_trunk_hight").as_double();
  map_size = this->get_parameter("map_size").as_double();
  cell_size = this->get_parameter("cell_size").as_double();
  max_rate = this->get_parameter("update_rate").as_double();

  std::string input_pointclouds_topic = this->get_parameter("input_pointclouds_topic").as_string();
  std::string output_pointclouds_topic = this->get_parameter("output_pointclouds_topic").as_string();

  // Создание объекта TrunkFinder и установка его параметров
  finder_ = new TrunkFinder;
  std::string err = finder_->setParameters(&finder_parameters);
  if (!err.empty()) {
    RCLCPP_WARN(this->get_logger(), "finder setting error: %s", err.c_str());
  }
  hight_ = finder_parameters["v_shift"];
  max_index = int(2 * map_size / cell_size);

  // Создание подписчика и издателя
  sub_input_clouds_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_pointclouds_topic, 2,
    std::bind(&TrunkDetector::inputCallback, this, std::placeholders::_1));
  pub_output_ = this->create_publisher<trunk_detector_msgs::msg::TrunkPoseArray>(output_pointclouds_topic, 2);
}

// Деструктор
TrunkDetector::~TrunkDetector()
{
  delete finder_;
}

// Обработчик входящих сообщений с облаками точек
void TrunkDetector::inputCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Received cloud with %d points", msg->width * msg->height);
  // Ограничение частоты обновления
  rclcpp::Duration duration = this->now() - update_time_;
  if (duration.seconds() < 1.0 / max_rate) {
    return;
  }
  update_time_ = this->now();

  trunk_detector_msgs::msg::TrunkPoseArray output_msg;
  std::map<int, std::vector<int>> results;
  pcl::fromROSMsg(*msg, *input_);
  std::string err = finder_->analyze(input_, output_, &results);
  if (!err.empty()) {
    RCLCPP_WARN(this->get_logger(), "analyze error: %s", err.c_str());
  }
  std::set<int> deleted;
  // Формирование сообщения с координатами и оценкой размера обнаруженных препятствий
  output_msg.header = msg->header;
  // обход результатов
  for (auto it_p = results.begin(); it_p != results.end(); ++it_p) {
    if (fabs(output_->points[it_p->first].x) >= map_size ||
        fabs(output_->points[it_p->first].y) >= map_size)
    {
      continue;
    }
    if (deleted.find(it_p->first) != deleted.end()) {
      continue;
    }
    trunk_detector_msgs::msg::TrunkPose trunk;
    index_search(it_p->first, results, trunk, deleted);
    trunk.x = (0.5 + floor(output_->points[it_p->first].x / cell_size)) * cell_size;
    trunk.y = (0.5 + floor(output_->points[it_p->first].y / cell_size)) * cell_size;
    category = medium;
    if (trunk.r < 0) {
      category = none;
    } else if (output_->points[it_p->first].z + hight_ < small_trunk_hight && trunk.r < 0.1) {
      category = small;
    } else if (output_->points[it_p->first].z + hight_ > medium_trunk_hight) {
      category = big;
    }
    trunk.c = static_cast<int8_t>(category);
    output_msg.trunks.push_back(trunk);
  }
  pub_output_->publish(output_msg);
  RCLCPP_INFO(this->get_logger(), "Publishing %ld trunks", output_msg.trunks.size());
}

// Для ROS2 достаточно rclcpp::spin(), поэтому метод run() не обязателен
void TrunkDetector::run()
{
  rclcpp::spin(shared_from_this());
}
