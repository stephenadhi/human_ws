from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    # get config file path and loading it
    config_file_path = os.path.join(
        get_package_share_directory('nav2_soloco_controller'),
        'config',
        'neural_motion_planner.yaml'
    )
    with open(config_file_path, 'r') as file:
        planner_config = yaml.safe_load(file)['nav2_soloco_controller']['ros__parameters']
    
    nav2_soloco_controller_cmd = Node(
        package='nav2_soloco_controller',
        executable='neural_motion_planner.py',
        name='neural_motion_planner',
        output='screen',
        parameters=[planner_config],
    )

    global_nav2_client_cmd = Node(
        package='nav2_soloco_controller',
        executable='global_nav2_client.py',
        name='global_nav2_client',
        output='screen',
        parameters=[planner_config],
    )

    subgoal_publisher_cmd = Node(
        package='nav2_soloco_controller',
        executable='subgoal_publisher',
        name='subgoal_publisher',
        output='screen',
        parameters=[planner_config],
    )

    ld = LaunchDescription()
    ld.add_action(global_nav2_client_cmd)
    ld.add_action(subgoal_publisher_cmd)
    ld.add_action(nav2_soloco_controller_cmd)

    return ld