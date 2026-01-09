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
        LaunchArg('rec_odom', default_value=['odom'], description='Received Odom message topic'),
        LaunchArg('imu', default_value=['imu'], description='Imu message topic'),
        LaunchArg('wheel', default_value=['wheel'], description='Imu message topic'),
        LaunchArg('namespace', default_value=['estimation_node']),

        Node(
            package='state_estimation',
            executable='estimation_node.py',
            name='estimation_node',
            namespace= LaunchConfig('namespace'),
            remappings=[('odom', LaunchConfig('rec_odom')),
                        ('imu', LaunchConfig('imu')),
                        ('wheel', LaunchConfig('wheel'))
                        ]#,
            #parameters=[{'T': LaunchConfig('T')}]
        )
    ])