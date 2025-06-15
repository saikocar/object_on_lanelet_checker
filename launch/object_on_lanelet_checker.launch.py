from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='object_on_lanelet_checker',
            executable='object_on_lanelet_checker_node',
            name='object_on_lanelet_checker',
            output='screen',
            parameters=[],
        )
    ])

