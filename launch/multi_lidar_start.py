
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('rslidar_sdk')
    config_path = os.path.join(pkg_share, 'config', 'multi_lidar_config.yaml')

    return LaunchDescription([
        Node(
            package='rslidar_sdk',
            executable='multi_lidar_node',
            name='multi_lidar_node',
            output='screen',
            parameters=[config_path],
            emulate_tty=True,
        )
    ])
