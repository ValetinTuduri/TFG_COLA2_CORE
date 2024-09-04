from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='stm32',
            executable='stm32_node',
            name='stm32',
            output='screen'
        ),
    ])
