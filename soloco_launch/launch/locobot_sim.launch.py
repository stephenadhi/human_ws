import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Create the launch configuration variables specific to simulation
    namespace = LaunchConfiguration('namespace')
    robot_model = LaunchConfiguration('robot_model')
    robot_name = LaunchConfiguration('robot_name')
    use_pedsim = LaunchConfiguration('use_pedsim')
    use_gazebo_gui = LaunchConfiguration('use_gazebo_gui')
    use_soloco_controller = LaunchConfiguration('use_soloco_controller')
    run_human_tf = LaunchConfiguration('run_human_tf')
    world_file = LaunchConfiguration('world_file')
    map_file = LaunchConfiguration('map_file')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    pedsim_scene_file = LaunchConfiguration('pedsim_scene_file')
    pedsim_config_file = LaunchConfiguration('pedsim_config_file')

    # Get the launch directory
    bringup_dir = get_package_share_directory('soloco_launch')
    interbotix_sim_dir = get_package_share_directory('interbotix_xslocobot_sim')
    nav2_soloco_controller_dir = get_package_share_directory('nav2_soloco_controller')
    pedsim_dir = get_package_share_directory('pedsim_simulator')

    default_world_path = os.path.join(bringup_dir, 'worlds', 'empty_world')
    default_map_path = os.path.join(pedsim_dir, 'maps', 'tb3_house_demo_crowd.yaml')
    default_nav_to_pose_bt_xml = LaunchConfiguration('default_nav_to_pose_bt_xml')
    default_nav_through_poses_bt_xml = LaunchConfiguration('default_nav_through_poses_bt_xml')
    default_pedsim_scene_path = os.path.join(pedsim_dir, 'scenarios', 'warehouse.xml')
    default_pedsim_config_path = os.path.join(pedsim_dir, 'config', 'params.yaml')

    declare_robot_model_cmd = DeclareLaunchArgument(
        'robot_model',
        default_value='locobot_base',
        description=('Interbotix LoCoBot model such as `locobot_base` or `locobot_wx250s`.'))

    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name',
        default_value='locobot',
        description=('name of the robot (could be anything but defaults to `locobot`).'))

    declare_use_pedsim_cmd = DeclareLaunchArgument(
        'use_pedsim',
        default_value='false',
        description='Whether to use pedestrian simulator')

    declare_use_gazebo_gui_cmd = DeclareLaunchArgument(
        'use_gazebo_gui',
        default_value='false',
        description='Whether to use lidar in simulator')

    declare_world_file_cmd = DeclareLaunchArgument(
        'world_file',
        default_value=default_world_path)

    declare_map_file_cmd = DeclareLaunchArgument(
        'map_file',
        default_value=default_map_path)

    declare_nav2_params_file_cmd = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('soloco_launch'),
            'params',
            'smac_dwb_nav2_params.yaml'
        ]),
        description=(
            'full path to the ROS 2 parameters file to use when configuring the Nav2 stack.'
        ),
    )

    declare_nav_to_pose_bt_xml = DeclareLaunchArgument(
        'default_nav_to_pose_bt_xml',
        default_value=os.path.join(
            get_package_share_directory('soloco_launch'),
            'config', 'behavior_trees', 'soloco_nav_to_pose_global.xml'),
        description='Full path to the behavior tree xml file to use')
    
    declare_nav_through_poses_bt_xml = DeclareLaunchArgument(
        'default_nav_through_poses_bt_xml',
        default_value=os.path.join(
            get_package_share_directory('soloco_launch'),
            'config', 'behavior_trees', 'soloco_nav_through_poses_global.xml'),
        description='Full path to the behavior tree xml file to use')

    declare_cmd_vel_topic_cmd = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value=('locobot/diffdrive_controller/cmd_vel_unstamped'),
        description="topic to remap /cmd_vel to.")

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

    declare_use_soloco_controller_cmd = DeclareLaunchArgument(
        'use_soloco_controller',
        default_value='False')
    
    declare_run_human_tf_cmd = DeclareLaunchArgument(
        'run_human_tf',
        default_value='False')

    simulator_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
          interbotix_sim_dir, 'launch', 'xslocobot_gz_classic.launch.py')),
        launch_arguments={
          'robot_model': robot_model,
          'robot_name':robot_name,
          'use_lidar': 'true',
          'use_rviz': 'false',
          'use_gazebo': use_gazebo_gui,
          'world_file_path': world_file,
        }.items())

    slam_toolbox_launch_cmd = TimerAction(
        period=10.0, # wait for simulator until launching nav2
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                bringup_dir, 'launch', 'nav2_slam_toolbox.launch.py')),
            launch_arguments={
            'cmd_vel_topic': cmd_vel_topic,
            'use_sim_time': 'true',
            'map': map_file,
            'nav2_params_file': nav2_params_file,
            'default_nav_to_pose_bt_xml': default_nav_to_pose_bt_xml,
            'default_nav_through_poses_bt_xml':default_nav_through_poses_bt_xml
            }.items())
        ])

    human_tf2_publisher_cmd = Node(
        package='soloco_perception',
        executable='human_tf2_publisher', # 'human_track_publisher',
        name='human_tf2_publisher',
        output='screen',
        condition=IfCondition(run_human_tf))

    pedsim_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
          pedsim_dir, 'launch', 'simulator_launch.py')),
        launch_arguments={
          'scene_file': pedsim_scene_file,
          'config_file': pedsim_config_file,
          'namespace': namespace,
        }.items(),
        condition=IfCondition(use_pedsim))

    pedsim_gazebo_spawner_cmd = Node(
        package='pedsim_gazebo_plugin',
        executable='spawn_pedsim_agents.py',
        name='spawn_pedsim_agents',
        output='screen',
        condition=IfCondition(use_pedsim))

    pedsim_tracker_cmd = Node(
        package='soloco_perception',
        executable='pedsim_tracks.py', # 'robot_track_publisher'
        name='pedsim_tracker',
        output='screen',
        condition=IfCondition(use_pedsim))

    robot_tracker_cmd = Node(
        package='soloco_perception',
        executable='robot_track.py', # 'robot_track_publisher'
        name='robot_tracker',
        output='screen',
        condition=IfCondition(use_soloco_controller))

    soloco_controller_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
          nav2_soloco_controller_dir, 'launch', 'social_planner.launch.py')),
        launch_arguments={
          'use_rviz': 'false',
        }.items(),
        condition=IfCondition(use_soloco_controller))

    multi_track_visualizer_cmd = Node(
        package='soloco_perception',
        executable='multi_track_visualizer.py', # 'robot_track_publisher'
        name='multi_track_visualizer',
        output='screen',
        condition=IfCondition(use_soloco_controller))

    # Create the launch description and populate
    ld = LaunchDescription()
    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_robot_model_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_use_pedsim_cmd)
    ld.add_action(declare_use_gazebo_gui_cmd)
    ld.add_action(declare_world_file_cmd)
    ld.add_action(declare_map_file_cmd)
    ld.add_action(declare_nav2_params_file_cmd)
    ld.add_action(declare_nav_to_pose_bt_xml)
    ld.add_action(declare_nav_through_poses_bt_xml)
    ld.add_action(declare_cmd_vel_topic_cmd)
    ld.add_action(declare_pedsim_scene_file_cmd)
    ld.add_action(declare_pedsim_config_file_cmd)
    ld.add_action(declare_run_human_tf_cmd)
    ld.add_action(declare_use_soloco_controller_cmd)
    # Add the actions to launch all of the nodes
    ld.add_action(simulator_launch_cmd)
    ld.add_action(slam_toolbox_launch_cmd)
    ld.add_action(pedsim_launch_cmd)
    ld.add_action(pedsim_gazebo_spawner_cmd)
    ld.add_action(human_tf2_publisher_cmd)
    ld.add_action(soloco_controller_launch_cmd)
    ld.add_action(multi_track_visualizer_cmd)
    ld.add_action(robot_tracker_cmd)

    return ld