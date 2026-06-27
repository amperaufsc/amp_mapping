from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument as LaunchArg,
    ExecuteProcess,
)
from launch.substitutions import LaunchConfiguration as LaunchConfig
from launch_ros.actions import LifecycleNode


def generate_launch_description():

    mapper_node = LifecycleNode(
        package='mapper',
        executable='mapper_node_life.py',
        name='mapper_node',
        namespace=LaunchConfig('namespace'),
        output='screen',

        remappings=[
            ('track', LaunchConfig('track')),
            ('odom', LaunchConfig('odom')),
            ('track_pub', LaunchConfig('track_pub')),
        ],
    )

    return LaunchDescription([

        LaunchArg(
            'namespace',
            default_value='mapper',
            description='Namespace for node'
        ),

        LaunchArg(
            'track',
            default_value='/perception/track',
            description='Detected track topic'
        ),

        LaunchArg(
            'odom',
            default_value='/orbslam3/odom',
            description='Odometry topic'
        ),

        LaunchArg(
            'track_pub',
            default_value='track_pub',
            description='Published track topic'
        ),

        mapper_node
    ])