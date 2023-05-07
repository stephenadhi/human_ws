import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Create the launch configuration variables specific to simulation
    namespace = LaunchConfiguration('namespace')
    robot_model = LaunchConfiguration('robot_model')
    robot_name = LaunchConfiguration('robot_name')
    use_gazebo_gui = LaunchConfiguration('use_gazebo_gui')
    world_file = LaunchConfiguration('world_file')
    map_file = LaunchConfiguration('map_file')
    pedsim_scene_file = LaunchConfiguration('pedsim_scene_file')
    pedsim_config_file = LaunchConfiguration('pedsim_config_file')

    # Get the launch directory
    bringup_dir = get_package_share_directory('soloco_launch')
    interbotix_sim_dir = get_package_share_directory('interbotix_xslocobot_sim')
    interbotix_nav_dir = get_package_share_directory('interbotix_xslocobot_nav')
    pedsim_dir = get_package_share_directory('pedsim_simulator')

    default_world_path = os.path.join(bringup_dir, 'worlds', 'empty_world')
    default_map_path = os.path.join(pedsim_dir, 'maps', 'tb3_house_demo_crowd.yaml')
    default_pedsim_scene_path = os.path.join(pedsim_dir, 'scenarios', 'warehouse.xml')
    default_pedsim_config_path = os.path.join(pedsim_dir, 'config', 'params.yaml')

    declare_robot_model_cmd = DeclareLaunchArgument(
        'robot_model',
        default_value='locobot_base',
        description=('Interbotix LoCoBot model such as `locobot_base` or `locobot_wx250s`.'))

    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_model',
        default_value='locobot',
        description=('name of the robot (could be anything but defaults to `locobot`).'))

    declare_use_gazebo_gui_cmd = DeclareLaunchArgument(
        'use_gazebo_gui',
        default_value='true',
        description='Whether to use lidar in simulator')

    declare_world_file_cmd = DeclareLaunchArgument(
        'world_file',
        default_value=default_world_path)

    declare_map_file_cmd = DeclareLaunchArgument(
        'map_file',
        default_value=default_map_path)

    declare_pedsim_scene_file_cmd = DeclareLaunchArgument(
        'pedsim_scene_file', 
        default_value=default_pedsim_scene_path,
        description='')

    declare_pedsim_config_file_cmd = DeclareLaunchArgument(
        'pedsim_config_file', 
        default_value=default_pedsim_config_path,
        description='')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    simulator_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
          interbotix_sim_dir, 'launch', 'xslocobot_gz_classic.launch.py')),
        launch_arguments={
          'robot_model': robot_model,
          'robot_name':robot_name,
          'use_lidar': 'true',
          'use_gazebo': use_gazebo_gui,
          'world_file_path': world_file,
        }.items())

    slam_toolbox_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
          interbotix_nav_dir, 'launch', 'xslocobot_slam_toolbox.launch.py')),
        launch_arguments={
          'launch_driver': 'false',
          'robot_name':robot_name,
          'use_lidar': 'true',
          'use_sim_time': 'true',
          'map': map_file,
        }.items())

    pedsim_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
          pedsim_dir, 'launch', 'simulator_launch.py')),
        launch_arguments={
          'scene_file': pedsim_scene_file,
          'config_file': pedsim_config_file,
          'namespace': namespace,
        }.items())

    pedsim_gazebo_spawner_cmd = Node(
        package='pedsim_gazebo_plugin',
        executable='spawn_pedsim_agents.py',
        name='spawn_pedsim_agents',
        output='screen')

    # Create the launch description and populate
    ld = LaunchDescription()
    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_robot_model_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_use_gazebo_gui_cmd)
    ld.add_action(declare_world_file_cmd)
    ld.add_action(declare_map_file_cmd)
    ld.add_action(declare_pedsim_scene_file_cmd)
    ld.add_action(declare_pedsim_config_file_cmd)
    # Add the actions to launch all of the nodes
    ld.add_action(simulator_launch_cmd)
    ld.add_action(slam_toolbox_launch_cmd)
    ld.add_action(pedsim_launch_cmd)
    ld.add_action(pedsim_gazebo_spawner_cmd)

    return ld