from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')

    # get config file path and loading it
    default_config_file_path = os.path.join(
        get_package_share_directory('nav2_soloco_controller'),
        'config',
        'neural_motion_planner.yaml'
    )
    with open(default_config_file_path, 'r') as file:
        soloco_config_file = yaml.safe_load(file)['nav2_soloco_controller']['ros__parameters']

    declare_cmd_vel_topic_cmd = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value=('locobot/commands/velocity'),
        description="topic to remap /cmd_vel to.")

    nav2_soloco_controller_cmd = Node(
        package='nav2_soloco_controller',
        executable='neural_motion_planner.py',
        name='neural_motion_planner',
        output='screen',
        parameters=[
            soloco_config_file,
            # Overriding cmd_vel_topic
            {'cmd_vel_topic': cmd_vel_topic}
        ],
    )

    subgoal_visualizer_cmd = Node(
        package='nav2_soloco_controller',
        executable='subgoal_visualizer',
        name='subgoal_visualizer',
        output='screen',
        parameters=[soloco_config_file],
    )

    ld = LaunchDescription()
    ld.add_action(declare_cmd_vel_topic_cmd)
    ld.add_action(subgoal_visualizer_cmd)
    ld.add_action(nav2_soloco_controller_cmd)

    return ld