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
        LaunchArg('namespace', default_value=['point'], description='Namespace for node'),
        LaunchArg('track', default_value=['track'], description='Track message topic'),
        LaunchArg('pointcloud', default_value=['/pointcloud'], description='Pointcloud message topic'),
        LaunchArg('frame_id', default_value=['/map'], description='Pointcloud frame'),
        LaunchArg('name', default_value=['name'], description='node name'),

        Node(
            package='as_utils',
            executable='pointcloud_rgb',
            name=LaunchConfig('name'),
            namespace= LaunchConfig('namespace'),
            remappings=[('/track', LaunchConfig('track')),
                        ('/pointcloud', LaunchConfig('pointcloud'))],
            parameters=[{'frame_id': LaunchConfig('frame_id')}]
        )
    ])