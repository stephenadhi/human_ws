from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    # get config file path and loading it
    tracker_config_file = os.path.join(
        get_package_share_directory('human_perception'),
        'config',
        'multi_tracker.yaml'
    )
    with open(human_tracker_file, 'r') as file:
        human_tracker_config = yaml.safe_load(file)['human_perception']['human_tracker']['ros__parameters']
        robot_tracker_config = yaml.safe_load(file)['human_perception']['robot_tracker']['ros__parameters']
    human_track_publisher_cmd = Node(
        package='human_perception',
        executable='human_track_publisher',
        name='human_track_publisher',
        output='screen',
        parameters=[human_tracker_config],
    )

    robot_track_publisher_cmd = Node(
        package='human_perception',
        executable='robot_track_publisher',
        name='robot_track_publisher',
        output='screen',
        parameters=[robot_tracker_config],
    )
    
    ld = LaunchDescription()
    
    ld.add_action(human_track_publisher_cmd)
    ld.add_action(robot_track_publisher_cmd)
    
    return ld