import os

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            LogInfo, RegisterEventHandler, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (Command, EnvironmentVariable, FindExecutable,
                                LaunchConfiguration, LocalSubstitution,
                                PythonExpression, PathJoinSubstitution)

os.environ['RCUTILS_COLORIZED_OUTPUT'] = '1'

def generate_launch_description():
    namespace = LaunchConfiguration('namespace')

    hostname = os.uname().nodename.replace('-', '_')
    namespace_launch_arg = DeclareLaunchArgument(
            'namespace',
            #default_value=EnvironmentVariable(name='HOSTNAME')
            default_value=hostname
            )

    log_level = LaunchConfiguration('log_level')
    declare_log_level_arg = DeclareLaunchArgument(
           'log_level', default_value='info',
            description='log level')

    nesfr7_arm_only_common_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('nesfr_arm_bringup'), 'launch',
                    'nesfr7_arm_only_common.launch.py'
                    ])
                ]),
            launch_arguments={
                'namespace': namespace,
                'log_level': log_level,
                }.items()
            )

    # joy stick
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
                ('joy/set_feedback', 'joy/set_feedback'),
                ],
            output='both',
            parameters=[joy_params])

    # TODO
#    nesfr_system_main = ExecuteProcess(
#            cmd=[[FindExecutable(name='nesfr_system')
#                ]],
#            shell=True
#            )

    return LaunchDescription([
        namespace_launch_arg,
        declare_log_level_arg,
        nesfr7_arm_only_common_launch,
        joy_node,
    ])
