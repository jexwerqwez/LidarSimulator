cmake_minimum_required(VERSION 3.16)
project(cloud_merger)

# --- найти зависимости ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters features registration)
find_package(tf2_sensor_msgs REQUIRED)

# --- включить заголовочные директории ---
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})

# --- библиотека с реализацией узла ---
add_library(cloud_merger_node
  src/cloud_merger_node.cpp
)
ament_target_dependencies(cloud_merger_node
  rclcpp
  sensor_msgs
  pcl_conversions
  pcl_ros
  tf2_ros
  tf2_sensor_msgs
  geometry_msgs
)
target_link_libraries(cloud_merger_node
  ${PCL_LIBRARIES}
)

# --- основной исполняемый файл ---
add_executable(cloud_merger_main
  src/cloud_merger_main.cpp
)
ament_target_dependencies(cloud_merger_main
  rclcpp
)
target_link_libraries(cloud_merger_main
  cloud_merger_node
)

# --- установка ---
install(
  TARGETS
    cloud_merger_node
    cloud_merger_main
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch
    config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
