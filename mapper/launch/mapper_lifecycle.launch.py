from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument as LaunchArg,
    ExecuteProcess,
)
from launch.substitutions import LaunchConfiguration as LaunchConfig
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():

    return LaunchDescription([
        LaunchArg('odom_pub',default_value=['/orbslam/odom'],description='odometry msg publisher'),
        LaunchArg('pose_sub',default_value=['/orbslam3/orbslam/pose'],description='pose msg subscriber'),
        LaunchArg('track_pub',default_value=['track_pub'],description='pose msg subscriber'),


    Node(
        package='mapper',
        executable='mapper_node_life.py',
        name='mapper_node',
        namespace=LaunchConfig('namespace'),
        output='screen',

        remappings=[
            ('track', LaunchConfig('track')),
            ('odom', LaunchConfig('odom')),
            ('track_pub', LaunchConfig('track_pub')),],
        )
    ])