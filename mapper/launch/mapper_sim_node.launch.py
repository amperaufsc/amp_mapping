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
        ExecuteProcess(
             cmd=['/opt/ros/humble/lib/tf2_ros/static_transform_publisher',
                  '--yaw', '-1.570796327',
                  '--roll', '-1.5707963270',
                  '--pitch', '0',
                  '--frame-id', '/fsds/cam2',
                  '--child-frame-id', 'fsds/map'],
             output='screen',),
        LaunchArg('namespace',default_value=['AMP'],description='namespace for Node'),
        LaunchArg('track',default_value=['/track_pub/trackstamped'],description='simulation track'),
        LaunchArg('odom',default_value=['/fsds/testing_only/odom'],description='simulation odom'),
        LaunchArg('track_pub',default_value=['/track_pub/track_map'],description='track msg after transform'),
        Node(
            package='mapper',
            executable='mapper_node.py',
            name='MaperNode',
            namespace=LaunchConfiguration('namespace'),
            remappings=[('track',LaunchConfiguration('track')),
                        ('odom',LaunchConfiguration('odom')),
                        ('track_pub',LaunchConfiguration('track_pub'))]
        )
    ])    