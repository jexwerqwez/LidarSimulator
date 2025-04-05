import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Пути к файлам параметров
    lidar_params = os.path.join(get_package_share_directory('lidar_simulation'), 'config', 'lidar_params.yaml')
    detector_params = os.path.join(get_package_share_directory('trunk_detector'), 'config', 'trunk_detector_param.yaml')
    rviz_config = os.path.join(get_package_share_directory('trunk_detector'), 'rviz', 'default_config.rviz')

    return LaunchDescription([
        Node(
            package='lidar_simulation',
            executable='lidar_simulation_node',
            name='lidar_simulation_node',
            output='screen',
            parameters=[lidar_params]
        ),
        # Узел классификатора стволов
        Node(
            package='trunk_detector',
            executable='trunk_detector_node',
            name='trunk_detector_node',
            output='screen',
            parameters=[
                detector_params,
                {
                    'input_pointclouds_topic': 'lidar/point_cloud',
                    'output_pointclouds_topic': '/detector_output'
                }
            ]
        ),

        # Узел визуализатора маркеров
        Node(
            package='trunk_detector',
            executable='trunk_marker_visualizer_node',
            name='trunk_marker_visualizer_node',
            output='screen',
            parameters=[
                {
                    'input_topic': '/detector_output',
                    'output_topic': '/trunk_markers'
                }
            ]
        )
    ])
