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

    static_tf = ExecuteProcess(
        cmd=[
            '/opt/ros/humble/lib/tf2_ros/static_transform_publisher',
            '--yaw', '-1.570796327',
            '--roll', '-1.570796327',
            '--pitch', '0',
            '--frame-id', 'left_camera_link',
            '--child-frame-id', 'oak_left_camera_optical_frame',
        ],
        output='screen',
    )

    return LaunchDescription([

        LaunchArg(
            'namespace',
            default_value='',
            description='Namespace for node'
        ),

        LaunchArg(
            'track',
            default_value='track',
            description='Detected track topic'
        ),

        LaunchArg(
            'odom',
            default_value='/fsds/testing_only/odom',
            description='Odometry topic'
        ),

        LaunchArg(
            'track_pub',
            default_value='track_pub',
            description='Published track topic'
        ),

        static_tf,

        mapper_node,
    ])