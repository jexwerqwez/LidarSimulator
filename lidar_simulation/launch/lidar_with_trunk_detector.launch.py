import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def ensure_cloud_directory(context, *args, **kwargs):
    cloud_dir = LaunchConfiguration('cloud_directory').perform(context)
    os.makedirs(cloud_dir, exist_ok=True)
    return []

def generate_launch_description():
    # Конфиги
    lidar_params = os.path.join(get_package_share_directory('lidar_simulation'), 'config', 'lidar_params.yaml')
    detector_params = os.path.join(get_package_share_directory('trunk_detector'), 'config', 'trunk_detector_param.yaml')
    rviz_config = os.path.join(get_package_share_directory('trunk_detector'), 'rviz', 'default_config.rviz')
    merger_params = os.path.join(get_package_share_directory('cloud_merger'), 'config', 'merger_params.yaml')

    cloud_dir_arg = DeclareLaunchArgument(
        'cloud_directory',
        default_value='/tmp/lidar_clouds',
        description='Directory containing .pcd files for cloud replay'
    )
    cloud_dir = LaunchConfiguration('cloud_directory')

    return LaunchDescription([
        cloud_dir_arg,
        OpaqueFunction(function=ensure_cloud_directory),  # создаём папку

        Node(
            package='lidar_simulation',
            executable='lidar_simulation_node',
            name='lidar_simulation_node',
            output='screen',
            parameters=[lidar_params]
        ),

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

        Node(
            package='trunk_detector',
            executable='trunk_marker_visualizer_node',
            name='trunk_marker_visualizer_node',
            output='screen',
            parameters=[{
                'input_topic': '/detector_output',
                'output_topic': '/trunk_markers'
            }]
        ),

        Node(
            package='cloud_replayer',
            executable='cloud_replayer_main',
            name='cloud_replayer_node',
            output='screen',
            parameters=[{
                'cloud_directory': cloud_dir,
                'replay_rate': 1.0
            }]
        ),

        Node(
            package='cloud_merger',
            executable='cloud_merger_main',
            name='cloud_merger_node',
            output='screen',
            parameters=[merger_params]
        )
    ])
