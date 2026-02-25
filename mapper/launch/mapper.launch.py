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
        'mapper_parameters.yaml'
    )

    return LaunchDescription([
        LaunchArg('namespace',default_value=['namespace'],description='namespace for Node'),
        LaunchArg('track',default_value=['track'],description='detected track'),
        LaunchArg('odom',default_value=['odom'],description='odometri msg'),
        LaunchArg('track_pub',default_value=['track_pub'],description='track msg after Luiz Lopez'),
        Node(
            package='mapper',
            executable='mapper_node.py',
            name='mapper_node',
            namespace=LaunchConfiguration('namespace'),
            remappings=[
                ('track', LaunchConfiguration('track')),
                ('odom', LaunchConfiguration('odom')), 
                ('track_pub', LaunchConfiguration('track_pub'))],
            parameters=[parameters_file],
        )
    ])