from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch.substitutions import LaunchConfiguration as LaunchConfig
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    parameters_file = os.path.join(
        get_package_share_directory('kalman_filter'),
        'config',
        'kalman_filter_parameters.yaml'
    )

    return LaunchDescription([
        LaunchArg('namespace', default_value=['kalman_filter'], description='Namespace for node'),
        LaunchArg('pose', default_value=['pose'], description='pose message topic'),
        LaunchArg('odometry', default_value=['odometry'], description='odometry message topic'),
        LaunchArg('imu', default_value=['imu'], description='imu message topic'),
        LaunchArg('ekf_odometry', default_value=['ekf_odometry'], description='EKF Odom message topic'),

        Node(
            package='kalman_filter',
            executable='kalman_filter_node.py',
            name='kalman_filter_node',
            namespace= LaunchConfig('namespace'),
            remappings=[('pose', LaunchConfig('pose')),
                        ('odometry', LaunchConfig('odometry')),
                        ('imu', LaunchConfig('imu')),
                        ('ekf_odometry', LaunchConfig('ekf_odometry'))],
            parameters=[parameters_file],
        )
    ])