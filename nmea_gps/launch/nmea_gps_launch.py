from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nmea_gps',
            executable='nmea_gps_node',
            name='nmea_gps',
            output='screen'
        ),
    ])
