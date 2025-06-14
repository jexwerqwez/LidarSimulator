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
    # пути до конфигов
    pkg_share = get_package_share_directory('lidar_simulation')
    multi_lidar_config = os.path.join(pkg_share, 'config', 'multi_lidar_simulation.yaml')
    detector_params   = os.path.join(get_package_share_directory('trunk_detector'), 'config', 'trunk_detector_param.yaml')
    rviz_config       = os.path.join(pkg_share, 'rviz', 'default_config.rviz')
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
            executable='multi_lidar_simulator',
            name='multi_lidar_simulator',
            output='screen',
            parameters=[{
                'config_file': multi_lidar_config
            }]
        ),

        Node(
            package='trunk_detector',
            executable='trunk_detector_node',
            name='trunk_detector_node',
            output='screen',
            parameters=[
                detector_params,
                {
                    'input_pointclouds_topic': '/merged_cloud',
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
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config]
        ),
        Node(
            package='cloud_merger',
            executable='cloud_merger_main',
            name='cloud_merger_node',
            output='screen',
            parameters=[merger_params]
        ),
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_twist_keyboard',
            output='screen',
            prefix='xterm -e',  # запускаем в новом окне терминала
        )

    ])
