from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('plvins_estimator'))
    return LaunchDescription([
        DeclareLaunchArgument('data_directory', description='External simulation directory containing cam_pose.txt, imu_pose.txt and keyframe/'),
        DeclareLaunchArgument('config_file', default_value=str(share / 'config/simdata/simdata_config_fix_extrinsic.yaml')),
        DeclareLaunchArgument('output_directory', default_value=str(Path.cwd() / 'results')),
        Node(package='plvins_estimator', executable='plvins_estimator', parameters=[{
            'config_file': LaunchConfiguration('config_file'), 'output_directory': LaunchConfiguration('output_directory')}], output='screen'),
        Node(package='pub_imu_features', executable='pub_imu_features', parameters=[{'sim_file_path': LaunchConfiguration('data_directory')}], output='screen'),
    ])
