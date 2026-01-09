from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch.actions import ExecuteProcess


def generate_launch_description():
    return LaunchDescription([
        LaunchArg('odom_pub',default_value=['orbslam/odom'],description='odometry msg publisher'),
        LaunchArg('pose_sub',default_value=['/orbslam3/orbslam/pose'],description='pose msg subscriber'),
        Node(
            package='ros2_mapper',
            executable='odometry_message.py',
            name='odometry_slam',
            remappings=[('odom_pub',LaunchConfiguration('odom_pub')),
                        ('pose_sub',LaunchConfiguration('pose_sub'))]
        )
    ])