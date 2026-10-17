from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('data_name', description='External EuRoC ground-truth CSV'),
        DeclareLaunchArgument('estimated_odometry', default_value='/plvins_estimator/odometry'),
        Node(package='benchmark_publisher', executable='benchmark_publisher', name='benchmark_publisher',
             parameters=[{'data_name': LaunchConfiguration('data_name')}],
             remappings=[('estimated_odometry', LaunchConfiguration('estimated_odometry'))], output='screen')])
