import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    # Package directories
    adis_imu_share_dir = get_package_share_directory('adis_imu')
    
    # Parameter file paths
    adis_imu_params = os.path.join(adis_imu_share_dir, 'config', 'adis_imu.yaml')
    imu_angle_estimator_params = os.path.join(adis_imu_share_dir, 'config', 'imu_angle_estimator.yaml')
    imu_calibration_params = os.path.join(adis_imu_share_dir, 'calibrations', 'adis_imu_last_calibration.yaml')
    
    return LaunchDescription([
        # Adis IMU Node
        Node(
            package='adis_imu',
            executable='adis_imu_node',
            name='adis_imu',
            namespace='adis_imu',
            output='screen',
            parameters=[adis_imu_params],
            respawn=True,
        ),
        
        # IMU Angle Estimator Node
       # Node(
       #     package='adis_imu',
       #     executable='imu_angle_estimator_node.py',
       #     name='imu_angle_estimator',
       #     namespace='imu_angle_estimator',
       #     output='screen',
       #     parameters=[imu_angle_estimator_params, imu_calibration_params],
       #     respawn=True,
       # ),
        
        # IMU Calibration Node
       # Node(
       #     package='adis_imu',
       #     executable='imu_calibration_node.py',
       #     name='imu_calibration',
       #     output='screen',
       #     respawn=True,
       # ),
    ])
