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
        LaunchArg('odometry', default_value=['odometry'], description='odometry message topic'),
        LaunchArg('imu', default_value=['imu'], description='imu message topic'),
        LaunchArg('ekf_odometry', default_value=['ekf_odometry'], description='EKF Odom message topic'),
        LaunchArg('freq_pub', default_value=['freq_pub'], description='message publishing frequency topic'),

        Node(
            package='kalman_filter',
            executable='ekf_sim_node.py',
            name='ekf_sim_node',
            namespace= LaunchConfig('namespace'),
            remappings=[('freq_pub', LaunchConfig('freq_pub')),
                        ('odometry', LaunchConfig('odometry')),
                        ('imu', LaunchConfig('imu')),
                        ('ekf_odometry', LaunchConfig('ekf_odometry'))],
            parameters=[parameters_file],
        )
    ])