import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Create the launch configuration variables
    use_soloco_controller = LaunchConfiguration('use_soloco_controller')   
    map_file = LaunchConfiguration('map_file')
    namespace = LaunchConfiguration('namespace')
    robot_model = LaunchConfiguration('robot_model')
    robot_name = LaunchConfiguration('robot_name')

    # Get the launch directory
    bringup_dir = get_package_share_directory('soloco_launch')
    soloco_perception_dir = get_package_share_directory('soloco_perception')
    nav2_soloco_controller_dir = get_package_share_directory('nav2_soloco_controller')

    default_map_path = os.path.join(bringup_dir, 'maps', 'tb3_house_demo_crowd.yaml')
    
    declare_robot_model_cmd = DeclareLaunchArgument(
        'robot_model',
        default_value='locobot_base',
        description=('Interbotix LoCoBot model such as `locobot_base` or `locobot_wx250s`.'))

    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name',
        default_value='locobot',
        description=('name of the robot (could be anything but defaults to `locobot`).'))

    declare_use_soloco_controller_cmd = DeclareLaunchArgument(
        'use_soloco_controller',
        default_value='False')
    
    declare_map_file_cmd = DeclareLaunchArgument(
        'map_file',
        default_value=default_map_path)
    
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    zed_perception_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
          soloco_perception_dir, 'launch', 'perception.launch.py')))

    zed_pointcloud_to_laserscan_launch_cmd = TimerAction(
        period=30.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    bringup_dir, 'launch', 'zed_pointcloud_to_laserscan.launch.py')))
        ]
    )

    nav2_bringup_slam_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
          bringup_dir, 'launch', 'nav2_slam_toolbox.launch.py')),
        condition=IfCondition(use_soloco_controller))
     
    # Create the launch description and populate
    ld = LaunchDescription()
    # Declare the launch options
    ld.add_action(declare_use_soloco_controller_cmd)
    ld.add_action(declare_map_file_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_robot_model_cmd)
    ld.add_action(declare_robot_name_cmd)

    # Add the actions to launch all of the nodes
    ld.add_action(zed_perception_launch_cmd)
    # ld.add_action(zed_pointcloud_to_laserscan_launch_cmd)
    ld.add_action(nav2_bringup_slam_launch_cmd)

    return ld