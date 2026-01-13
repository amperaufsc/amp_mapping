from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch.actions import ExecuteProcess
from launch import LaunchDescription
from launch.actions import TimerAction, LogInfo, EmitEvent, RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition, TransitionEvent

class Respawn:
    
    def callback(self, event, context):
        return [
            LogInfo(msg='Enviando shutdown pro MAPPER.'),
            EmitEvent(
                event=ChangeState(
                    lifecycle_node_matcher=lambda node: node == lifecycle_node,
                    transition_id=Transition.TRANSITION_UNCONFIGURED_SHUTDOWN
                )
            )
                
        ]

def generate_launch_description():
    global lifecycle_node
    lifecycle_node = LifecycleNode(
        package='ros2_mapper',
        executable='mapper_lifecycle.py',
        name='mapper_node',
        namespace='',
        respawn=False,
        output='screen',
        remappings=[
                ('track', LaunchConfiguration('track')),
                ('odom', LaunchConfiguration('odom')),
                ('track_pub', LaunchConfiguration('track_pub'))
            ]
    )

    #counter = Respawn()

    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=lambda node: node == lifecycle_node,
            transition_id=Transition.TRANSITION_CONFIGURE
        )
    )
    activate_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=lambda node: node == lifecycle_node,
            transition_id=Transition.TRANSITION_ACTIVATE
        )
    )
    on_exit_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=lifecycle_node,
            on_exit=[LogInfo(msg="Mapper finalizando")]
        )
    )
    return LaunchDescription([
        
        LaunchArg('namespace',default_value=['namespace'],description='namespace for Node'),
        LaunchArg('track',default_value=['track'],description='detected track'),
        LaunchArg('odom',default_value=['/fsds/testing_only/odom'],description='odometri msg'),
        LaunchArg('track_pub',default_value=['track_pub'],description='track msg after Luiz Lopez'),
        ExecuteProcess(
             cmd=['/opt/ros/humble/lib/tf2_ros/static_transform_publisher',
                  '--yaw', '-1.570796327',
                  '--roll', '-1.5707963270',
                  '--pitch', '0',
                  '--frame-id', 'left_camera_link',
                  '--child-frame-id', 'oak_left_camera_optical_frame'],
             output='screen',),
        lifecycle_node,
        TimerAction(period=12.0, actions=[configure_event,LogInfo(msg='Configurando Mapper')]),
        TimerAction(period=14.0, actions=[activate_event,LogInfo(msg='Ativando Mapper')]),
        on_exit_handler,
    ])