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
        LaunchArg('namespace',default_value=['namespace'],description='namespace for Node'),
        LaunchArg('track',default_value=['track'],description='simulation track'),
        LaunchArg('odom',default_value=['odom'],description='simulation odom'),
        LaunchArg('track_pub',default_value=['track_pub'],description='track msg after transform'),
        Node(
            package='mapper',
            executable='sim_track_node.py',
            name='sim_mapper_node',
            namespace=LaunchConfiguration('namespace'),
            remappings=[
                ('track', LaunchConfiguration('track')),
                ('odom', LaunchConfiguration('odom')), 
                ('track_pub', LaunchConfiguration('track_pub'))],
            parameters=[parameters_file],
        )
    ])