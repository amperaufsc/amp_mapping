from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument as LaunchArg
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    parameters_file = os.path.join(
        get_package_share_directory('control'),
        'config',
        'control_parameters.yaml'
    )

    return LaunchDescription([
        LaunchArg('namespace',default_value=['namespace'],description='namespace for Node'),
        LaunchArg('track',default_value=['track'],description='detected track'),
        LaunchArg('odom',default_value=['odom'],description='odometri msg'),
        LaunchArg('track_pub',default_value=['track_pub'],description='track msg after Luiz Lopez'),


        Node(
            package='mapper',  # Substitua pelo nome correto do seu pacote
            executable='sim_track_node',  # Substitua pelo nome correto do seu executável
            name='sim_track_node',
            output='screen',
            parameters=[{
                'apply_error': True  # Ou False, se desejar desabilitar a aplicação de erro
            }]
        
        )
    ])
