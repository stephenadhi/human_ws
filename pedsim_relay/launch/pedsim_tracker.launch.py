from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def generate_launch_description():
    # Get the launch directory
    pkg_dir = get_package_share_directory('pedsim_relay')

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')

    declare_namespace_cmd = DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Top-level namespace')

    # get config file path and loading it
    human_config_file = os.path.join(
        get_package_share_directory('soloco_perception'),
        'config',
        'human_tracker.yaml'
    )
    with open(human_config_file, 'r') as file:
        human_tracker_config = yaml.safe_load(file)['soloco_perception']['human_tracker']['ros_parameters']

    # Interpolate human trajectory
    pedsim_tracker_cmd = Node(
        package='pedsim_relay',
        executable='pedsim_tracks.py',
        name='pedsim_tracks',
        output='screen',
        parameters=[human_tracker_config],
    )

    # Only send human data when inside local costmap
    pedsim_relay_cmd = Node(
        package='pedsim_relay',
        executable='pedsim_relay_node', 
        name='pedsim_relay_node',
        output='screen',
    )

    # Create the launch description and populate
    ld = LaunchDescription()
    # Declare the launch options
    ld.add_action(declare_namespace_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(pedsim_tracker_cmd)
    ld.add_action(pedsim_relay_cmd)

    return ld