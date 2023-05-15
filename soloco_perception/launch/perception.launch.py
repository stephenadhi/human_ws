import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('soloco_perception')
    launch_dir = os.path.join(bringup_dir, 'launch')

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')

    declare_namespace_cmd = DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Top-level namespace')

    camera_launch_cmd = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(launch_dir, 'zed2.launch.py')),
    )

    tracker_launch_cmd = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(launch_dir, 'multi_tracker.launch.py')),
    )

    # Create the launch description and populate
    ld = LaunchDescription()
    # Declare the launch options
    ld.add_action(declare_namespace_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(camera_launch_cmd)
    ld.add_action(tracker_launch_cmd)

    return ld