cmake_minimum_required(VERSION 3.16)
project(cloud_replayer)

# ROS 2 и PCL зависимости
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)

# Включаем include директории
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

# Либра с реализацией узла
add_library(${PROJECT_NAME}_node
  src/cloud_replayer_node.cpp
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp 
  sensor_msgs 
  pcl_conversions 
  pcl_ros 
  tf2_ros 
  geometry_msgs
  visualization_msgs
)
target_link_libraries(${PROJECT_NAME}_node ${PCL_LIBRARIES})

# Main (точка входа)
add_executable(cloud_replayer_main src/cloud_replayer_main.cpp)
target_link_libraries(cloud_replayer_main ${PROJECT_NAME}_node)
ament_target_dependencies(cloud_replayer_main
  rclcpp
)

# Установка бинарников
install(TARGETS
  ${PROJECT_NAME}_node
  cloud_replayer_main
  DESTINATION lib/${PROJECT_NAME}
)

# Установка ресурсов
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
