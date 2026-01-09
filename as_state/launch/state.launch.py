from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch.actions import ExecuteProcess


def generate_launch():
    return LaunchDescription([
        Node(
            package='as_state',
            executable='state_node.py',
            name='state_node',
            remappings=[
                # ('ebs', 'std_msgs/ebs'),
                # ('ts_active', 'std_msgs/ts_active'),
                # ('r2d', 'std_msgs/r2d'),
                # ('brake', 'std_msgs/brake'),
                # ('sdc', 'std_msgs/sdc'),
                # ('MF', 'std_msgs/MF'),
                # ('state', 'std_msgs/state')
            ]
        )
    ])