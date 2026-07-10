from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration as LaunchConfig

def generate_launch_description():

    return LaunchDescription([

        Node(
            package='transformation_broadcast',
            executable='dynamic_transformation.py',
            name='transformation_broadcast',
            parameters=[],
            output='screen',
            remappings=[('odom', LaunchConfig('odom'))]
        ),
    ])
