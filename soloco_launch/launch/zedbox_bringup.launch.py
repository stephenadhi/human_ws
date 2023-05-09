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
    launch_neural_planner = LaunchConfiguration('launch_neural_planner')   
    map_file = LaunchConfiguration('map_file')
    namespace = LaunchConfiguration('namespace')
    robot_model = LaunchConfiguration('robot_model')
    robot_name = LaunchConfiguration('robot_name')

    # Get the launch directory
    bringup_dir = get_package_share_directory('soloco_launch')
    human_perception_dir = get_package_share_directory('human_perception')
    social_motion_planner_dir = get_package_share_directory('social_motion_planner')

    default_map_path = os.path.join(bringup_dir, 'maps', 'tb3_house_demo_crowd.yaml')
    
    declare_robot_model_cmd = DeclareLaunchArgument(
        'robot_model',
        default_value='locobot_base',
        description=('Interbotix LoCoBot model such as `locobot_base` or `locobot_wx250s`.'))

    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name',
        default_value='locobot',
        description=('name of the robot (could be anything but defaults to `locobot`).'))

    declare_launch_neural_planner_cmd = DeclareLaunchArgument(
        'launch_neural_planner',
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
          human_perception_dir, 'launch', 'perception.launch.py')))

    zed_pointcloud_to_laserscan_launch_cmd = TimerAction(
        period=30.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    bringup_dir, 'launch', 'zed_pointcloud_to_laserscan.launch.py')))
        ]
    )


    social_motion_planner_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
          social_motion_planner_dir, 'launch', 'social_planner.launch.py')),
        launch_arguments={
          'use_rviz': 'false',
        }.items(),
        condition=IfCondition(launch_neural_planner))
     
    # Create the launch description and populate
    ld = LaunchDescription()
    # Declare the launch options
    ld.add_action(declare_launch_neural_planner_cmd)
    ld.add_action(declare_map_file_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_robot_model_cmd)
    ld.add_action(declare_robot_name_cmd)

    # Add the actions to launch all of the nodes
    ld.add_action(zed_perception_launch_cmd)
    ld.add_action(zed_pointcloud_to_laserscan_launch_cmd)
    ld.add_action(social_motion_planner_launch_cmd)

    return ld