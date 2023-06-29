# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
This launch script borrows heavily from the original Nav2 bringup launch file:
    https://github.com/ros-planning/navigation2/blob/humble/nav2_bringup/launch/bringup_launch.py
"""
import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    GroupAction
)
from launch.conditions import (
    IfCondition,
    UnlessCondition,
    LaunchConfigurationEquals,
    LaunchConfigurationNotEquals,
)
from launch.substitutions import (
    PathJoinSubstitution,
    LaunchConfiguration,
    TextSubstitution,
)
from launch_ros.actions import (
    Node,
    SetParameter
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import HasNodeParams, RewrittenYaml


def launch_setup(context, *args, **kwargs):
    namespace_launch_arg = LaunchConfiguration('namespace')
    use_sim_time_launch_arg = LaunchConfiguration('use_sim_time')
    log_level_launch_arg = LaunchConfiguration('log_level')
    autostart_launch_arg = LaunchConfiguration('autostart')
    use_composition_launch_arg = LaunchConfiguration('use_composition')
    use_respawn_launch_arg = LaunchConfiguration('use_respawn')
    nav2_params_file_launch_arg = LaunchConfiguration('nav2_params_file')
    slam_toolbox_params_file_launch_arg = LaunchConfiguration('slam_toolbox_params_file')
    use_slam_toolbox_launch_arg = LaunchConfiguration('use_slam_toolbox')
    map_yaml_file_launch_arg = LaunchConfiguration('map')
    default_nav_to_pose_bt_xml_launch_arg = LaunchConfiguration('default_nav_to_pose_bt_xml')
    default_nav_through_poses_bt_xml_launch_arg = LaunchConfiguration('default_nav_through_poses_bt_xml')
    # Set env var to print messages to stdout immediately
    set_logging_env_var = SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    lifecycle_nodes_navigation = [
        'planner_server',
        'controller_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower'
    ]

    lifecycle_nodes_slam = [
        'map_saver'
    ]

    lifecycle_nodes_localization = [
        'map_server'
        'amcl'
    ]

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    tf_remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'
        )
    ]

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time_launch_arg,
        'autostart': autostart_launch_arg,
        'default_nav_to_pose_bt_xml' : default_nav_to_pose_bt_xml_launch_arg,
        'default_nav_through_poses_bt_xml' : default_nav_through_poses_bt_xml_launch_arg
    }

    configured_params = RewrittenYaml(
        source_file=nav2_params_file_launch_arg,
        root_key=namespace_launch_arg,
        param_rewrites=param_substitutions,
        convert_types=True,
    )

    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[
            configured_params
        ],
        remappings=tf_remappings,
    )

    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        parameters=[
            configured_params
        ],
        remappings=tf_remappings,
    )

    behavior_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[
            configured_params
        ],
        remappings=tf_remappings,
    )

    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[
            configured_params
        ],
        remappings=tf_remappings,
    )

    waypoint_follower_node = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[
            configured_params
        ],
        remappings=tf_remappings,
    )

    lifecycle_manager_navigation_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time_launch_arg},
            {'autostart': autostart_launch_arg},
            {'node_names': lifecycle_nodes_navigation},
        ]
    )

    slam_toolbox_online_sync_slam_node = Node(
        condition=LaunchConfigurationEquals('slam_toolbox_mode', 'online_sync'),
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[
            slam_toolbox_params_file_launch_arg,
            {
                'use_sim_time': use_sim_time_launch_arg,
            }
        ],
        output='screen'
    )

    slam_toolbox_online_async_slam_node = Node(
        condition=LaunchConfigurationEquals('slam_toolbox_mode', 'online_async'),
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[
            slam_toolbox_params_file_launch_arg,
            {
                'use_sim_time': use_sim_time_launch_arg,
            }
        ],
        output='screen'
    )

    slam_toolbox_localization_slam_node = Node(
        condition=LaunchConfigurationEquals('slam_toolbox_mode', 'localization'),
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[
            slam_toolbox_params_file_launch_arg,
            {
                'use_sim_time': use_sim_time_launch_arg,
            }
        ],
        output='screen'
    )

    slam_toolbox_nav2_nodes = GroupAction(
        condition=IfCondition(use_slam_toolbox_launch_arg),
        actions=[
            SetParameter('use_sim_time', use_sim_time_launch_arg),
            Node(
                condition=IfCondition(use_composition_launch_arg),
                name='nav2_container',
                package='rclcpp_components',
                executable='component_container_isolated',
                parameters=[configured_params, {'autostart': autostart_launch_arg}],
                arguments=['--ros-args', '--log-level', log_level_launch_arg],
                remappings=tf_remappings,
                output='screen'),
            Node(
                condition=LaunchConfigurationNotEquals('slam_toolbox_mode', 'localization'),
                package='nav2_map_server',
                executable='map_saver_server',
                output='screen',
                respawn=use_respawn_launch_arg,
                respawn_delay=2.0,
                arguments=[
                    '--ros-args', '--log-level', log_level_launch_arg
                ],
                parameters=[
                    configured_params
                ],
            ),
            Node(
                condition=LaunchConfigurationEquals('slam_toolbox_mode', 'localization'),
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                respawn=use_respawn_launch_arg,
                respawn_delay=2.0,
                arguments=[
                    '--ros-args', '--log-level', log_level_launch_arg
                ],
                parameters=[
                    configured_params,
                    {'yaml_filename': map_yaml_file_launch_arg},
                ],
                remappings=tf_remappings
            ),
            Node(
                condition=LaunchConfigurationEquals('slam_toolbox_mode', 'localization'),
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
                respawn=use_respawn_launch_arg,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level_launch_arg],
                remappings=tf_remappings
            ),
            Node(
                condition=LaunchConfigurationNotEquals('slam_toolbox_mode', 'localization'),
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_slam',
                output='screen',
                arguments=[
                    '--ros-args', '--log-level', log_level_launch_arg
                ],
                parameters=[
                    {'autostart': autostart_launch_arg},
                    {'node_names': lifecycle_nodes_slam},
                ]
            ),
            Node(
                condition=LaunchConfigurationEquals('slam_toolbox_mode', 'localization'),
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                arguments=[
                    '--ros-args', '--log-level', log_level_launch_arg
                ],
                parameters=[
                    {'autostart': autostart_launch_arg},
                    {'node_names': lifecycle_nodes_localization},
                ]
            ),
        ]
    )

    return [
        set_logging_env_var,
        planner_server_node,
        controller_server_node,
        behavior_server_node,
        bt_navigator_node,
        waypoint_follower_node,
        lifecycle_manager_navigation_node,
        slam_toolbox_online_sync_slam_node,
        slam_toolbox_online_async_slam_node,
        slam_toolbox_localization_slam_node,
        slam_toolbox_nav2_nodes,
    ]

def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'default_nav_to_pose_bt_xml',
            default_value=os.path.join(
                get_package_share_directory('soloco_launch'),
                'config', 'behavior_trees', 'soloco_nav_to_pose_global.xml'),
            description='Full path to the behavior tree xml file to use'
        )
    ),
    declared_arguments.append(
        DeclareLaunchArgument(
            'default_nav_through_poses_bt_xml',
            default_value=os.path.join(
                get_package_share_directory('soloco_launch'),
                'config', 'behavior_trees', 'soloco_nav_through_poses_global.xml'),
            description='Full path to the behavior tree xml file to use'
        )
    ),
    declared_arguments.append(
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='top-level namespace',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            choices=('true', 'false'),
            description='automatically startup the Nav2 stack.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_composition',
            default_value='true',
            choices=('true', 'false'),
            description='Whether to use composed bringup',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
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
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_slam_toolbox',
            default_value='true',
            choices=('true', 'false'),
            description=(
                'whether to use slam_toolbox over rtabmap.'
            )
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'slam_toolbox_mode',
            default_value='online_async',
            choices=(
                # 'lifelong',
                'localization',
                # 'offline',
                'online_async',
                'online_sync'
            ),
            description=(
                "the node to launch the SLAM in using the slam_toolbox. Currently only "
                "'localization', 'online_sync', and 'online_async' modes are supported."
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'slam_toolbox_params_filename',
            default_value=('slam_toolbox_', LaunchConfiguration('slam_toolbox_mode'), '.yaml')
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'slam_toolbox_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare("soloco_launch"),
                'config',
                LaunchConfiguration('slam_toolbox_params_filename'),
            ]),
            description='full path to the ROS 2 parameters file to use for the slam_toolbox node',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            choices=('true', 'false'),
            description=(
                'tells ROS nodes asking for time to get the Gazebo-published simulation time, '
                'published over the ROS topic /clock; this value is automatically set to `true` if'
                ' using Gazebo hardware.'
            )
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'map',
            default_value='',
            description='Full path to map yaml file to load if using localization mode'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            choices=('debug', 'info', 'warn', 'error', 'fatal'),
            description='set the logging level of the Nav2 nodes.'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_respawn', default_value='false',
            choices=('true', 'false'),
            description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
