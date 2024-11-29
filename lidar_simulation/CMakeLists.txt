cmake_minimum_required(VERSION 3.16)
project(lidar_simulation)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)

include_directories(
  ${PROJECT_SOURCE_DIR}/lidar_simulation/include
)

add_executable(${PROJECT_NAME}_node
  src/Objects/Plane.cpp
  src/Objects/Sphere.cpp
  src/Lidar.cpp
  src/Visualization.cpp
  src/LidarController.cpp
  src/Main.cpp
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

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
