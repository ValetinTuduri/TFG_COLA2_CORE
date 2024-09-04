import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():

    dvl_share_dir = get_package_share_directory('linkquest_navquest600_dvl')
     # Parameter file paths
    dvl_params = os.path.join(dvl_share_dir, 'config', 'linkquest_navquest600_dvl.yaml')

    return LaunchDescription([

        Node(
            package='linkquest_navquest600_dvl',
            executable='linkquest_navquest600_dvl_node',
            name='linkquest_navquest600_dvl',
            output='screen',
            parameters=[dvl_params]
        )
    ])
