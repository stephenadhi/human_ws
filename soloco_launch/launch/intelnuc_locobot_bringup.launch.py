import os
import yaml
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration, 
    PythonExpression,
    PathJoinSubstitution,
    LaunchConfiguration,
    NotSubstitution
)
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Create the launch configuration 
    use_nav2_slam = LaunchConfiguration('use_nav2_slam')
    use_soloco_controller = LaunchConfiguration('use_soloco_controller')
    run_human_tf = LaunchConfiguration('run_human_tf')
    map_file = LaunchConfiguration('map_file')
    namespace = LaunchConfiguration('namespace')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    robot_model = LaunchConfiguration('robot_model')
    robot_name = LaunchConfiguration('robot_name')

    # Get the launch directory
    bringup_dir = get_package_share_directory('soloco_launch')
    soloco_perception_dir = get_package_share_directory('soloco_perception')
    nav2_soloco_controller_dir = get_package_share_directory('nav2_soloco_controller')
    interbotix_nav_dir = get_package_share_directory('interbotix_xslocobot_nav')

    default_map_path = os.path.join(bringup_dir, 'maps', 'tb3_house_demo_crowd.yaml')
    # get config file path and loading it
    neural_config_file_path = os.path.join(
        nav2_soloco_controller_dir, 'config', 'neural_motion_planner.yaml')
    with open(neural_config_file_path, 'r') as file:
        planner_config = yaml.safe_load(file)['nav2_soloco_controller']['ros__parameters']

    declare_use_nav2_slam_cmd = DeclareLaunchArgument(
            'use_nav2_slam',
            default_value='False')
        
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
    
    declare_run_human_tf_cmd = DeclareLaunchArgument(
        'run_human_tf',
        default_value='False')
    
    declare_map_file_cmd = DeclareLaunchArgument(
        'map_file',
        default_value=default_map_path)

    declare_nav2_params_filename_cmd = DeclareLaunchArgument(
        'nav2_params_filename',
        default_value='smac_mppi_nav2_params.yaml')

    declare_nav2_params_file_cmd = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare("soloco_launch"),
            'params',
            LaunchConfiguration('nav2_params_filename')]))

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    robot_track_publisher_cmd = Node(
        package='soloco_perception',
        executable='robot_track.py', 
        name='robot_track_publisher',
        output='screen',
        condition=IfCondition(use_soloco_controller)
    )

    multi_track_visualizer_cmd = Node(
        package='soloco_perception',
        executable='multi_track_visualizer.py', 
        name='multi_track_visualizer',
        output='screen',
        condition=IfCondition(use_soloco_controller)
    )

    human_tf2_publisher_cmd = Node(
        package='soloco_perception',
        executable='human_tf2_publisher',
        name='human_tf2_publisher',
        output='screen',
        condition=IfCondition(run_human_tf))

    locobot_nav2_bringup_slam_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
          interbotix_nav_dir, 'launch', 'xslocobot_slam_toolbox.launch.py')),
        launch_arguments={
          'cmd_vel_topic': '/locobot/commands/velocity',
          'launch_driver': 'true',
          # 'map': map_file,
          'nav2_params_file': nav2_params_file,
          'robot_model':robot_model,
          'robot_name':robot_name,
          'slam_toolbox_mode': 'online_async',
          'use_camera': 'false', # Not using Intel camera
          'use_lidar': 'true',
          'lidar_type': 'rplidar_a2m8',
          'use_rviz': 'false',
          'use_sim_time': 'false',
        }.items(),
        condition=IfCondition(NotSubstitution(use_soloco_controller)))

    xslocobot_control_launch_cmd = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('interbotix_xslocobot_control'),
                'launch',
                'xslocobot_control.launch.py',
            ])
        ]),
        launch_arguments={
            'robot_name': 'locobot',
            'use_lidar': 'true',
            'lidar_type': 'rplidar_a2m8',
            'use_rviz': 'false',
            'rviz_frame': 'map',
            'use_camera': 'false',
            'rs_camera_align_depth': 'true',
            'use_base': 'true',
            # 'use_dock': 'true',
            'use_sim_time': 'false',
        }.items(),
        condition=IfCondition(use_soloco_controller))
    
    nav2_bringup_slam_launch_cmd  = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
          bringup_dir, 'launch', 'nav2_slam_toolbox.launch.py')),
        condition=IfCondition(use_nav2_slam))

    soloco_controller_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
          nav2_soloco_controller_dir, 'launch', 'social_planner.launch.py')),
        launch_arguments={
          'cmd_vel_topic': 'locobot/commands/velocity',
        }.items(),
        condition=IfCondition(use_soloco_controller))

    # Create the launch description and populate
    ld = LaunchDescription()
    # Declare the launch options
    ld.add_action(declare_use_nav2_slam_cmd)
    ld.add_action(declare_use_soloco_controller_cmd)
    ld.add_action(declare_run_human_tf_cmd)
    ld.add_action(declare_map_file_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_nav2_params_filename_cmd)
    ld.add_action(declare_nav2_params_file_cmd)
    ld.add_action(declare_robot_model_cmd)
    ld.add_action(declare_robot_name_cmd)

    # Add the actions to launch all of the nodes
    ld.add_action(robot_track_publisher_cmd)
    ld.add_action(multi_track_visualizer_cmd)
    ld.add_action(human_tf2_publisher_cmd)
    ld.add_action(xslocobot_control_launch_cmd)
    ld.add_action(locobot_nav2_bringup_slam_launch_cmd)
    ld.add_action(soloco_controller_launch_cmd)
    ld.add_action(nav2_bringup_slam_launch_cmd)

    return ld