cmake_minimum_required(VERSION 3.16)
project(lidar_simulation)

# ROS 2 и другие зависимости
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(std_srvs REQUIRED)
# YAML-парсер
find_package(yaml-cpp REQUIRED)
# Добавление директорий заголовочных файлов
include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${YAML_CPP_INCLUDE_DIRS}
)

# ------------------------------------------------------------
# 1) Сборка общей библиотеки lidar_simulation_library:
#    в ней лежат Plane, Sphere, Cylinder, Lidar, Visualization
# ------------------------------------------------------------
add_library(lidar_simulation_library
  src/Objects/Plane.cpp
  src/Objects/Sphere.cpp
  src/Objects/Cylinder.cpp
  src/Lidar.cpp
  src/Visualization.cpp
)

# Линковка зависимостей для библиотеки
ament_target_dependencies(lidar_simulation_library
  rclcpp
  sensor_msgs
  pcl_conversions
  pcl_ros
  tf2_ros
  geometry_msgs
  visualization_msgs
  std_srvs
  yaml-cpp
)

# ------------------------------------------------------------
# 2) Собираем исполняемый узел multi_lidar_simulator
# ------------------------------------------------------------
add_executable(multi_lidar_simulator
  src/multi_lidar_node.cpp
  src/MultiLidarSimulatorNode.cpp
)

target_link_libraries(multi_lidar_simulator
  lidar_simulation_library
  ${YAML_CPP_LIBRARIES}
)

ament_target_dependencies(multi_lidar_simulator
  rclcpp
  sensor_msgs
  pcl_conversions
  pcl_ros
  tf2_ros
  geometry_msgs
  visualization_msgs
  yaml-cpp
)

# find_package(GTest REQUIRED)

# add_executable(tests
#   test/TestMain.cpp
#   test/FunctionalTest.cpp
#   )

# add_executable(tests
#   test/TestMain.cpp
#   test/IntegrationTest.cpp
#   )

# add_executable(tests
#   test/TestMain.cpp
#   test/StressTest.cpp
#   )

# target_link_libraries(tests lidar_simulation_library GTest::GTest GTest::Main)

# ------------------------------------------------------------
# 3) Установка
# ------------------------------------------------------------
install(
  TARGETS
    lidar_simulation_library
    multi_lidar_simulator
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()