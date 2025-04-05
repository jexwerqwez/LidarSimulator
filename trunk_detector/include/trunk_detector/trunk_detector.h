/*  The trunk_finder package
**  TrunkDetector class header file
**  Created on February 19 2018
*/

#ifndef TRUNK_DETECTOR_H
#define TRUNK_DETECTOR_H

#include <map>
#include <set>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"

#include "trunk_detector_msgs/msg/trunk_pose.hpp"
#include "trunk_detector_msgs/msg/trunk_pose_array.hpp"

#include "trunk_detector/trunk_finder.h"  // класс TrunkFinder (ROS2 версия не требует изменений)

typedef pcl::PointXYZI PointT;

// enum для классов препятствий
enum TrunkCategory {
  unknown = 0,
  none = 1,
  small = 2,
  medium = 3,
  big = 4
};

// Класс детектора для работы с ROS-интерфейсом
class TrunkDetector : public rclcpp::Node
{
public:
  // конструктор
  explicit TrunkDetector(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  // деструктор
  ~TrunkDetector();

private:
  // обработчик для входящих сообщений с облаками точек
  void inputCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  // рекурсивная функция поиска связанных точек
  void index_search(int index, std::map<int, std::vector<int>> & container,
                    trunk_detector_msgs::msg::TrunkPose & trunk, std::set<int> & checked);
  // основной цикл детектора (не требуется, если используем rclcpp::spin)
  void run();

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_input_clouds_;
  rclcpp::Publisher<trunk_detector_msgs::msg::TrunkPoseArray>::SharedPtr pub_output_;

  rclcpp::Time update_time_;  // время последнего входящего сообщения

  TrunkFinder* finder_;  // указатель на объект для поиска препятствий
  pcl::PointCloud<PointT>::Ptr input_;           // входное облако
  pcl::PointCloud<PointT>::Ptr output_;          // выходное облако

  TrunkCategory category;  // класс препятствия

  double max_rate;         // предельная частота обновления
  double hight_;           // высота лидара от земли
  double small_trunk_hight;  // порог для "малых" препятствий
  double medium_trunk_hight; // порог для "средних" препятствий
  double map_size;           // радиус области видимости
  double cell_size;          // размер ячейки карты
  int max_index;             // число ячеек по одной стороне
};

#endif // TRUNK_DETECTOR_H
