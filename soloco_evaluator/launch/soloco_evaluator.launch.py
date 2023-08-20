
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    metrics_file = LaunchConfiguration('metrics_file')
    # agent configuration file
    metrics_file_path = os.path.join(get_package_share_directory('soloco_evaluator'),
        'config',
        'metrics.yaml'
    )

    declare_metrics_cmd = DeclareLaunchArgument(
        'metrics_file', default_value='metrics.yaml',
        description='Specify the name of the metrics configuration file in the cofig directory'
    )

    with open(metrics_file_path, 'r') as file:
        eval_config = yaml.safe_load(file)['hunav_evaluator_node']['ros__parameters']

    soloco_evaluator_node = Node(
        package='soloco_evaluator',
        executable='soloco_evaluator_node.py',
        name='soloco_evaluator_node',
        output='screen',
        parameters=[eval_config]
    )

    prediction_evaluator_node = Node(
        package='soloco_evaluator',
        executable='prediction_evaluator',
        name='prediction_evaluator_node',
        output='screen',
        parameters=[eval_config]
    )

    ld = LaunchDescription()
    ld.add_action(declare_metrics_cmd)
    ld.add_action(soloco_evaluator_node)
    ld.add_action(prediction_evaluator_node)
   
    return ld
    

    