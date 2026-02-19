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
        LaunchArg('Imu_sub', default_value=['Imu_sub'], description='Received Odom message topic'),
        LaunchArg('Ins_sub', default_value=['Ins_sub'], description='Imu message topic'),
        LaunchArg('Attitude_sub', default_value=['Attitude_sub'], description='Imu message topic'),


        Node(
            package='state_estimation',
            executable='vectornav_msgs_node.py',
            name='vectornav_msgs_node',
            remappings=[('Imu_sub', LaunchConfig('Imu_sub')),
                        ('Ins_sub', LaunchConfig('Ins_sub')),
                        ('Attitude_sub', LaunchConfig('Attitude_sub'))]
            )
        ]
    )