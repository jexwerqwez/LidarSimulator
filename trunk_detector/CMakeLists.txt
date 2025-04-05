cmake_minimum_required(VERSION 3.5)
project(trunk_detector)

# Установка стандарта C++
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Поиск зависимостей
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trunk_detector_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

# Включаем заголовочные файлы
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

# Узел классификатора
add_executable(trunk_detector_node
  src/trunk_detector_node.cpp
  src/trunk_detector.cpp
  src/trunk_finder.cpp
)

# Подключение зависимостей для целевого файла
ament_target_dependencies(trunk_detector_node
  rclcpp
  sensor_msgs
  std_msgs
  trunk_detector_msgs
  pcl_conversions
  pcl_ros
)

# Линкуем библиотеки PCL
target_link_libraries(${PROJECT_NAME}_node
  ${PCL_LIBRARIES}
)

# Узел визуализации
add_executable(trunk_marker_visualizer_node
  src/trunk_marker_visualizer_node.cpp
)

ament_target_dependencies(trunk_marker_visualizer_node
  rclcpp
  std_msgs
  visualization_msgs
  trunk_detector_msgs
)


# Установка бинарников
install(TARGETS
  trunk_detector_node
  trunk_marker_visualizer_node
  DESTINATION lib/${PROJECT_NAME}
)

# Установка ресурсов
install(DIRECTORY
  launch
  config
  rviz
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
