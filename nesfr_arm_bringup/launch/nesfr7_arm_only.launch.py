import os

from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            LogInfo, RegisterEventHandler, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (EnvironmentVariable, FindExecutable,
                                LaunchConfiguration, LocalSubstitution,
                                PythonExpression, PathJoinSubstitution)

def generate_launch_description():
    namespace = LaunchConfiguration('namespace')

    hostname = os.uname().nodename.replace('-', '_')
    namespace_launch_arg = DeclareLaunchArgument(
            'namespace',
            #default_value=EnvironmentVariable(name='HOSTNAME')
            default_value=hostname
            )


    nesfr7_arm_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('nesfr_arm_bringup'), 'launch',
                    'nesfr7_arm_common.launch.py'
                    ])
                ]),
            launch_arguments={
                'namespace': namespace
                }.items()
            )

    joy_params = {
            'device_id': 0,
            'device_name': "",
            'deadzone': 0.5,
            'autorepeat_rate': 20.0,
            #                    'sticky_buttons': 'false',
            'coalesce_interval_ms': 1
            }
    joy_node = Node(
            package='joy',
            executable='joy_node',
            namespace=namespace,
            remappings = [
                ('joy', 'joy'),
                ('joy/set_feedback', 'xbox_joy/set_feedback'),
                ],
            output='both',
            parameters=[joy_params])

    # TODO
    nesfr_system_main = ExecuteProcess(
            cmd=[[FindExecutable(name='nesfr_system')
                ]],
            shell=True
            )

    return LaunchDescription([
        namespace_launch_arg,
        nesfr7_arm_launch,
        joy_node,
        # TODO
        #nesfr_system_main,
    ])
