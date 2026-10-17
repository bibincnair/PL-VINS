from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('plvins_estimator'))
    config = LaunchConfiguration('config_file')
    common = {'config_file': config,
              'output_directory': LaunchConfiguration('output_directory'),
              'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=str(share / 'config/euroc/loop.yaml')),
        DeclareLaunchArgument('output_directory', default_value=str(Path.cwd() / 'results')),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('loop_closure', default_value='true'),
        DeclareLaunchArgument('overlay', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        Node(package='feature_tracker', executable='feature_tracker', name='feature_tracker', parameters=[common], output='screen'),
        Node(package='feature_tracker', executable='LineFeature_tracker', name='linefeature_tracker', parameters=[common], output='screen'),
        Node(package='plvins_estimator', executable='plvins_estimator', name='plvins_estimator', parameters=[common], output='screen'),
        Node(package='pose_graph', executable='pose_graph', name='pose_graph', parameters=[common], output='screen', condition=IfCondition(LaunchConfiguration('loop_closure'))),
        Node(package='image_node_b', executable='image_node_b', name='image_node_b', parameters=[common], output='screen', condition=IfCondition(LaunchConfiguration('overlay'))),
        Node(package='rviz2', executable='rviz2', arguments=['-d', str(share / 'config/plvins.rviz')],
             parameters=[{'use_sim_time': common['use_sim_time']}], condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
