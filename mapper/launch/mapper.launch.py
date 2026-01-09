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
        LaunchArg('namespace',default_value=['namespace'],description='namespace for Node'),
        LaunchArg('track',default_value=['track'],description='detected track'),
        LaunchArg('odom',default_value=['odom'],description='odometri msg'),
        LaunchArg('track_pub',default_value=['track_pub'],description='track msg after Luiz Lopez'),
        Node(
            package='ros2_mapper',
            executable='mapper_node.py',
            name='mapper_node',
            namespace=LaunchConfiguration('namespace'),
            remappings=[
                ('track', LaunchConfiguration('track')),
                ('odom', LaunchConfiguration('odom')), 
                ('track_pub', LaunchConfiguration('track_pub')),
            ]
        )
    ])