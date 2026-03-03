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
        LaunchArg('Ekf_odom_sub', default_value=['Ekf_odom_sub'], description='Received ekf Odom message topic'),
        LaunchArg('fsds_odom_sub', default_value=['fsds_odom_sub'], description='Received fsds Odom message topic'),
        LaunchArg('odom_pub', default_value=['odom_pub'], description='Odom published message topic'),

        Node(
            package='state_estimation',
            executable='ekf_odom_sim_node.py',
            name='ekf_odom_sim_node',
            remappings=[('Ekf_odom_sub', LaunchConfig('Ekf_odom_sub')),
                        ('fsds_odom_sub', LaunchConfig('fsds_odom_sub')),
                        ('odom_pub', LaunchConfig('odom_pub'))
                        ]
        )
    ])