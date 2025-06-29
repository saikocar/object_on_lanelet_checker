from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('object_on_lanelet_checker'),
        'config',
        'config.yaml'
    )

    return LaunchDescription([
        Node(
            package='object_on_lanelet_checker',
            executable='object_on_lanelet_checker_node',
            name='object_on_lanelet_checker',
            output='screen',
            parameters=[config_path],
        )
    ])

