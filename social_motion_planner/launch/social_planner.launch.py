from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    # get config file path and loading it
    config_file_path = os.path.join(
        get_package_share_directory('social_motion_planner'),
        'config',
        'local_planner.yaml'
    )
    with open(config_file_path, 'r') as file:
        planner_config = yaml.safe_load(file)['social_motion_planner']['ros__parameters']
    
    social_motion_planner_cmd = Node(
        package='social_motion_planner',
        executable='neural_motion_planner',
        name='neural_motion_planner',
        output='screen',
        parameters=[planner_config],
    )
    
    ld = LaunchDescription()
    
    ld.add_action(social_motion_planner_cmd)
    return ld