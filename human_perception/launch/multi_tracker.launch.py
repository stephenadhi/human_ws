from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    config_rviz2 = os.path.join(
        get_package_share_directory('human_perception'),
        'rviz',
        'multi_tracking.rviz'
    )
    # get config file path and loading it
    human_config_file = os.path.join(
        get_package_share_directory('human_perception'),
        'config',
        'human_tracker.yaml'
    )
    robot_config_file = os.path.join(
        get_package_share_directory('human_perception'),
        'config',
        'robot_tracker.yaml'
    )
    with open(human_config_file, 'r') as file:
        human_tracker_config = yaml.safe_load(file)['human_perception']['human_tracker']['ros_parameters']

    with open(robot_config_file, 'r') as file:
        robot_tracker_config = yaml.safe_load(file)['human_perception']['robot_tracker']['ros_parameters']

    human_track_publisher_cmd = Node(
        package='human_perception',
        executable='human_tracks.py', # 'human_track_publisher',
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

    multi_tracking_rviz2_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='multitracking_rviz',
        output='screen',
        arguments=[["-d"], [config_rviz2]],
    )        
        
    
    ld = LaunchDescription()
    
    ld.add_action(human_track_publisher_cmd)
    # ld.add_action(robot_track_publisher_cmd)
    ld.add_action(multi_tracking_rviz2_cmd)
    
    return ld