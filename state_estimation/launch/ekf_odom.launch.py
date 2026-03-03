from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration as LaunchConfig


def generate_launch_description():

    return LaunchDescription([
        LaunchArg('Odom_sub', default_value=['Odom_sub'], description='Received ekf Odom message topic'),
        LaunchArg('Imu_sub', default_value=['Imu_sub'], description='Received Imu message topic'),
        LaunchArg('Attitude_sub', default_value=['Attitude_sub'], description='Received Attitude message topic'),
        LaunchArg('odom_pub', default_value=['odom_pub'], description='Published Odom message topic'),

        Node(
            package='state_estimation',
            executable='ekf_odom_node.py',
            name='ekf_odom_node',
            remappings=[('Odom_sub', LaunchConfig('Odom_sub')),
                        ('Imu_sub', LaunchConfig('Imu_sub')),
                        ('Attitude_sub', LaunchConfig('Attitude_sub')),
                        ('odom_pub', LaunchConfig('odom_pub'))
                        ]
        )
    ])