from launch import LaunchDescription
import launch
import launch_ros.parameter_descriptions
import launch_ros.actions
import os
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(get_package_share_directory('cola2_nav'), 'config', 'navigator.yaml')
    print(f"Loading config from: {config}")
        
    node=Node(package = 'cola2_nav',name = 'navigator_surface_node',executable = 'navigator_surface_node', parameters = [config])
    ld.add_action(node)
    return ld
