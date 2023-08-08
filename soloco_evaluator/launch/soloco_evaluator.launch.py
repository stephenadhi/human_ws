
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    metrics_file_name = LaunchConfiguration('metrics_file')
    # agent configuration file
    metrics_file = PathJoinSubstitution([
        FindPackageShare('soloco_evaluator'),
        'config',
        metrics_file_name
    ])

    declare_metrics_cmd = DeclareLaunchArgument(
        'metrics_file', default_value='metrics.yaml',
        description='Specify the name of the metrics configuration file in the cofig directory'
    )

    soloco_evaluator_node = Node(
        package='soloco_evaluator',
        executable='soloco_evaluator_node.py',
        name='soloco_evaluator_node',
        output='screen',
        parameters=[metrics_file]
    )

    prediction_evaluator_node = Node(
        package='soloco_evaluator',
        executable='prediction_evaluator',
        name='prediction_evaluator_node',
        output='screen',
        parameters=[metrics_file]
    )

    ld = LaunchDescription()
    ld.add_action(declare_metrics_cmd)
    ld.add_action(soloco_evaluator_node)
    ld.add_action(prediction_evaluator_node)
   
    return ld
    

    