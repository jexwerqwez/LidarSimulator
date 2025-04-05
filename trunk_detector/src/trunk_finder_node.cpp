#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>


#include "trunk_detector/trunk_finder.h"

#include <map>
#include <vector>
#include <string>
#include <ctime>
#include <cstdio>

// Для удобства
using PointT = pcl::PointXYZI;

class TrunkFinderNode : public rclcpp::Node
{
public:
  TrunkFinderNode()
  : Node("trunk_finder_node"), write_flag_(false), counter_(0), exit_flag_(false),
    color_(cloud_, 250, 100, 100)
  {
    // Задаём параметры ноды (например, v_shift)
    this->declare_parameter("v_shift", 1.7);
    this->set_parameter(rclcpp::Parameter("v_shift", 1.7));

    // Если передан аргумент для записи, можно инициализировать write_flag_ и file_name_
    // (в данном примере аргументы из командной строки не обрабатываются, но их можно добавить)

    // Инициализация объекта TrunkFinder
    // (Параметры можно также передать через setParameters)
    
    // Инициализация PCL визуализатора
    viewer_ = std::make_shared<pcl::visualization::PCLVisualizer>("3D Viewer");

    // Создание паблишера и подписчика
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("trunks", 1);
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/velodyne_points", 1,
      std::bind(&TrunkFinderNode::pointcloud_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Subscribed to /velodyne_points");
  }

private:
  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (!write_flag_) {
      // Очистка визуализатора
      viewer_->removeAllShapes();
      viewer_->removeAllPointClouds();
      results_.clear();

      unsigned time_ = clock();
      sensor_msgs::msg::PointCloud2 output_msg;
      pcl::fromROSMsg(*msg, *cloud_);
      std::string err = finder_.analyze(cloud_, output_, &results_);
      pcl::toROSMsg(*output_, output_msg);
      publisher_->publish(output_msg);
      time_ = clock() - time_;
      // Можно добавить вывод времени обработки, если нужно

      // Визуализация
      viewer_->addPointCloud<PointT>(cloud_, color_, "cloud");
      viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
      int count = 0;
      char name[255];
      for (auto it = results_.begin(); it != results_.end(); ++it) {
        for (auto it_v = it->second.begin(); it_v != it->second.end(); ++it_v) {
          count++;
          sprintf(name, "line_%d", count);
          viewer_->addLine<PointT>(output_->points[it->first], output_->points[*it_v], 0, 0, 0, name);
        }
      }
      viewer_->spinOnce(100);
      results_.clear();
    } else {
      counter_++;
      if (counter_ == 12) {
        pcl::fromROSMsg(*msg, *cloud_);
        finder_.analyze(cloud_, output_, &results_);
        pcl::io::savePCDFileASCII(file_name_, *cloud_);
        RCLCPP_INFO(this->get_logger(), "recorded!");
        exit_flag_ = true;
      }
    }
    if (exit_flag_) {
      rclcpp::shutdown();
    }
  }

  // Объекты и переменные
  TrunkFinder          finder_;
  std::map<int, std::vector<int>> results_;
  pcl::PointCloud<PointT>::Ptr cloud_{new pcl::PointCloud<PointT>};
  pcl::PointCloud<PointT>::Ptr output_{new pcl::PointCloud<PointT>};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  std::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;
  pcl::visualization::PointCloudColorHandlerCustom<PointT> color_;
  bool write_flag_;
  bool exit_flag_;
  unsigned counter_;
  std::string file_name_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TrunkFinderNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
