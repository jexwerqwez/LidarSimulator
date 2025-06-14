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
find_package(std_srvs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(GTest REQUIRED)
find_package(ament_cmake_gtest REQUIRED)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${YAML_CPP_INCLUDE_DIRS}
)

add_library(lidar_simulation_library
  src/Objects/Plane.cpp
  src/Objects/Sphere.cpp
  src/Objects/Cylinder.cpp
  src/Lidar.cpp
  src/Visualization.cpp
)

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

add_executable(lidar_control_node
  src/LidarKeyboardController.cpp
)

ament_target_dependencies(lidar_control_node
  rclcpp
  geometry_msgs
)

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

ament_add_gtest(tests
  test/TestMain.cpp
  # test/FunctionalTest.cpp
  # test/IntegrationTest.cpp
  test/StressTest.cpp
)

target_link_libraries(tests
  lidar_simulation_library
)

ament_target_dependencies(tests
  rclcpp
  sensor_msgs
  geometry_msgs
  pcl_conversions
  pcl_ros
)

install(
  TARGETS
    lidar_simulation_library
    multi_lidar_simulator
    lidar_control_node
    tests
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()