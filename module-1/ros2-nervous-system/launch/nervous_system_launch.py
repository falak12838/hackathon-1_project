from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_nervous_system',
            executable='nervous_system_node',
            name='nervous_system_node',
            output='screen',
            parameters=[
                {'param_name': 'param_value'}
            ]
        )
    ])