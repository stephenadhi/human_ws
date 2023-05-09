from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            name='scanner', default_value='zed2',
            description='Namespace for sample topics'
        ),
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[('cloud_in', [LaunchConfiguration(variable_name='scanner'), '/zed_node/point_cloud/cloud_registered']),
                        ('scan', [LaunchConfiguration(variable_name='scanner'), '/zed_node/projected_scan'])],
            parameters=[{
                'target_frame': 'locobot/odom',
                'transform_tolerance': 0.3,
                'min_height': 0.0,
                'max_height': 1.5,
                'angle_min': -1.5708,  # -M_PI/2
                'angle_max': 1.5708,  # M_PI/2
                'angle_increment': 0.0087,  # M_PI/360.0
                'scan_time': 0.1,
                'range_min': 0.2,
                'range_max': 5.0,
                'use_inf': True,
                # 'inf_epsilon': 1.0
            }],
            name='pointcloud_to_laserscan'
        )
    ])