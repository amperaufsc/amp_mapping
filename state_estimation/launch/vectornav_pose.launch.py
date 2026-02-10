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
        LaunchArg('input_pose', default_value=['input_pose'], description='Received pose topic'),
        LaunchArg('output_pose', default_value=['output_pose'], description='Corrected pose topic'),
        
        Node(
            package='state_estimation',
            executable='vectornav_pose_node',
            name='vectornav_pose_node',
            #namespace= LaunchConfig('namespace'),
            remappings=[('input_pose', LaunchConfig('input_pose')),
                        ('output_pose', LaunchConfig('output_pose')),
                        ])
    ])