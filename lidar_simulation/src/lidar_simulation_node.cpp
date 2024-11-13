#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/LinearMath/Quaternion.h>

#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>

class LidarSimulationNode : public rclcpp::Node {
 public:
  LidarSimulationNode() : Node("lidar_simulation_node") {
    // инициализация параметров для лидара и плоскостей
    this->declare_parameter<double>("lidar_height",
                                    1.0);  // высота лидара над землёй
    this->declare_parameter<int>("num_lasers",
                                 30);  // количество лазеров по вертикали
    this->declare_parameter<double>("alpha_begin", -45.0);  // начальный угол
    this->declare_parameter<double>("alpha_end", 45.0);  // конечный угол
    this->declare_parameter<double>("laser_range", 20.0);  // дальность лазера
    this->declare_parameter<std::string>("lidar_frame", "lidar_frame");

    // параметры для плоскостей
    this->declare_parameter<std::vector<double>>("planes_tilt_angles",
                                                 {20.0, -20.0});
    this->declare_parameter<std::vector<double>>("planes_heights", {1.0, 1.5});

    // создание топиков для публикации точек и маркеров
    point_cloud_publisher_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>("lidar_points",
                                                              10);
    marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "visualization_marker", 10);

    generateRays();       // генерация лучей
    generatePlanes();     // создание плоскостей
    findIntersections();  // поиск пересечений

    // таймер для публикации данных
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&LidarSimulationNode::publishData, this));
  }

 private:
  struct Plane {
    Eigen::Vector3d normal;  // нормаль плоскости
    double height;           // высота плоскости
  };

  std::vector<Eigen::Vector3d> rays_;  // направления всех лучей
  std::vector<Plane> planes_;  // список всех плоскостей
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_;

  // функция для генерации лучей с учетом заданных углов
  void generateRays() {
    double alpha_begin = this->get_parameter("alpha_begin").as_double();
    double alpha_end = this->get_parameter("alpha_end").as_double();
    int num_lasers = this->get_parameter("num_lasers").as_int();

    // переводим углы в радианы и рассчитываем шаг между лазерами
    double alpha_begin_rad = alpha_begin * M_PI / 180.0;
    double alpha_end_rad = alpha_end * M_PI / 180.0;
    double vertical_angle_step =
        (alpha_end_rad - alpha_begin_rad) / (num_lasers - 1);

    // циклы для создания всех лучей, проходящих вокруг по горизонтали и с
    // заданными вертикальными углами
    for (double yaw = 0.0; yaw < 2 * M_PI;
         yaw += (2 * M_PI) /
                360.0) {  // проходим круг по горизонтали с шагом в один градус
      for (int i = 0; i < num_lasers;
           ++i) {  // проходим по всем лазерным вертикальным углам
        double vertical_angle =
            alpha_begin_rad +
            i * vertical_angle_step;  // высчитываем текущий вертикальный угол
        rays_.emplace_back(
            cos(vertical_angle) *
                cos(yaw),  // координата x с учетом вертикального и
                           // горизонтального угла
            cos(vertical_angle) *
                sin(yaw),  // координата y с учетом вертикального и
                           // горизонтального угла
            sin(vertical_angle));  // координата z по вертикальному углу
      }
    }
  }

  // функция для создания наклонных плоскостей
  void generatePlanes() {
    auto planes_tilt_angles =
        this->get_parameter("planes_tilt_angles").as_double_array();
    auto planes_heights =
        this->get_parameter("planes_heights").as_double_array();

    // проверяем, совпадает ли количество углов с количеством высот
    if (planes_tilt_angles.size() != planes_heights.size()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Количество углов наклона не соответствует количеству высот "
                   "плоскостей.");
      return;
    }

    // создаем каждую плоскость, вычисляя нормаль и высоту
    for (size_t i = 0; i < planes_tilt_angles.size(); ++i) {
      double tilt_angle_rad = planes_tilt_angles[i] * M_PI / 180.0;
      Plane plane;
      plane.normal =
          Eigen::Vector3d(sin(tilt_angle_rad), 0, cos(tilt_angle_rad));
      plane.height = planes_heights[i];
      planes_.push_back(plane);
    }
  }

  // функция для поиска точек пересечения лучей с плоскостями
  void findIntersections() {
    double laser_range = this->get_parameter("laser_range").as_double();
    cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());

    // для каждого луча ищем ближайшую точку пересечения
    for (const auto& ray : rays_) {
      Eigen::Vector3d nearest_intersection;
      double min_distance = laser_range;

      for (size_t i = 0; i < planes_.size(); ++i) {
        const auto& plane = planes_[i];
        double denominator = ray.dot(plane.normal);
        if (fabs(denominator) >
            1e-6) {  // проверяем, что луч не параллелен плоскости
          double t = -plane.height / denominator;
          if (t > 0 && t <= laser_range) {
            Eigen::Vector3d intersection_point = t * ray;
            if (isWithinBounds(intersection_point, i)) {
              double distance = intersection_point.norm();
              if (distance < min_distance) {
                min_distance = distance;
                nearest_intersection = intersection_point;
              }
            }
          }
        }
      }

      // сохраняем ближайшую точку в облако точек
      if (min_distance < laser_range) {
        pcl::PointXYZI point;
        point.x = nearest_intersection.x();
        point.y = nearest_intersection.y();
        point.z = nearest_intersection.z();
        point.intensity = min_distance;
        cloud_->points.push_back(point);
      }
    }

    // настраиваем параметры облака точек
    cloud_->width = static_cast<uint32_t>(cloud_->points.size());
    cloud_->height = 1;
    cloud_->is_dense = false;
  }

  // функция для проверки, находится ли точка внутри допустимых границ
  bool isWithinBounds(const Eigen::Vector3d& point,
                      size_t current_plane_index) {
    for (size_t i = 0; i < planes_.size(); ++i) {
      if (i == current_plane_index) continue;

      const auto& other_plane = planes_[i];
      double side = point.dot(other_plane.normal) + other_plane.height;
      if (side > 0) {
        // точка выходит за пределы плоскости
        return false;
      }
    }
    return true;
  }

  // функция для публикации облака точек и маркеров
  void publishData() {
    publishPointCloud();
    publishPlaneMarkers();
  }

  // публикация облака точек в топик
  void publishPointCloud() {
    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*cloud_, output);
    output.header.frame_id = this->get_parameter("lidar_frame").as_string();
    output.header.stamp = this->now();
    point_cloud_publisher_->publish(output);
  }

  // публикация маркеров плоскостей для визуализации
  void publishPlaneMarkers() {
    for (size_t i = 0; i < planes_.size(); ++i) {
      visualization_msgs::msg::Marker plane_marker;
      plane_marker.header.frame_id =
          this->get_parameter("lidar_frame").as_string();
      plane_marker.header.stamp = this->now();
      plane_marker.ns = "lidar_plane";
      plane_marker.id = static_cast<int>(i);
      plane_marker.type = visualization_msgs::msg::Marker::CUBE;
      plane_marker.action = visualization_msgs::msg::Marker::ADD;

      plane_marker.pose.position.x = 0.0;
      plane_marker.pose.position.y = 0.0;
      plane_marker.pose.position.z = planes_[i].height / 2.0;

      tf2::Quaternion orientation;
      orientation.setRPY(0, asin(planes_[i].normal.x()), 0);
      plane_marker.pose.orientation.x = orientation.x();
      plane_marker.pose.orientation.y = orientation.y();
      plane_marker.pose.orientation.z = orientation.z();
      plane_marker.pose.orientation.w = orientation.w();

      plane_marker.scale.x = 10.0;
      plane_marker.scale.y = 10.0;
      plane_marker.scale.z = 0.01;
      plane_marker.color.r = 0.0f;
      plane_marker.color.g = 0.0f;
      plane_marker.color.b = 1.0f;
      plane_marker.color.a = 0.5f;

      marker_publisher_->publish(plane_marker);
    }
  }

  // указатели на публикуемые топики
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      point_cloud_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      marker_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarSimulationNode>());
  rclcpp::shutdown();
  return 0;
}
