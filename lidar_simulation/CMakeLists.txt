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

# Добавление директорий заголовочных файлов
include_directories(
  ${PROJECT_SOURCE_DIR}/include
)

# Создание общей библиотеки
add_library(lidar_simulation_library
  src/Objects/Plane.cpp
  src/Objects/Sphere.cpp
  src/Lidar.cpp
  src/Visualization.cpp
  src/LidarController.cpp
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
)

add_executable(${PROJECT_NAME}_node src/Main.cpp)
target_link_libraries(${PROJECT_NAME}_node lidar_simulation_library)


find_package(GTest REQUIRED)

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

# Установка файлов
install(TARGETS
  lidar_simulation_library
  ${PROJECT_NAME}_node
  # tests
  DESTINATION lib/${PROJECT_NAME}
)

# Упаковка для ROS 2
ament_package()
