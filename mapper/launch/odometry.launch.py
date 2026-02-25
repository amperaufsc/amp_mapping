from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch.actions import ExecuteProcess
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    parameters_file = os.path.join(
        get_package_share_directory('control'),
        'config',
        'control_parameters.yaml'
    )

    return LaunchDescription([
        LaunchArg('odom_pub',default_value=['orbslam/odom'],description='odometry msg publisher'),
        LaunchArg('pose_sub',default_value=['/orbslam3/orbslam/pose'],description='pose msg subscriber'),
        Node(
            package='mapper',
            executable='odometry_message.py',
            name='odometry_slam',
            remappings=[('odom_pub',LaunchConfiguration('odom_pub')),
                        ('pose_sub',LaunchConfiguration('pose_sub'))],
            parameters=[parameters_file],
        )
    ])