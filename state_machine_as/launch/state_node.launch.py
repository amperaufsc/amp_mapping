from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='state_machine_as',
            executable='state_node',
            name='state_machine_node',
            output='screen',
            parameters=[],
            emulate_tty=True,
        )
    ])
