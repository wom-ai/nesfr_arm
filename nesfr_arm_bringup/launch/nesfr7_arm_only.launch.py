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

def generate_launch_description():
    namespace = LaunchConfiguration('namespace')

    hostname = os.uname().nodename.replace('-', '_')
    namespace_launch_arg = DeclareLaunchArgument(
            'namespace',
            #default_value=EnvironmentVariable(name='HOSTNAME')
            default_value=hostname
            )

    #
    # robot_state_publisher_node
    #
    nesfr_arm_params = PathJoinSubstitution(
        [FindPackageShare("nesfr_arm_description"), "config", "nesfr7_arm.yaml"]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("nesfr_arm_description"), "urdf", "nesfr_arm.urdf.xacro"]),
            " ",
            "nesfr_arm_params:=",
            nesfr_arm_params,
            " ",
            "prefix:=",
            hostname + '/',
            " ",
        ]
    )
    robot_description = {"robot_description": robot_description_content}
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=namespace,
        output="both",
        parameters=[robot_description],
    )


    #
    # nesfr_arm_node
    #
    robot_config_file = LaunchConfiguration('robot_config_file', default=[namespace, '.yaml'])
    nesfr7_arm_params = PathJoinSubstitution(
            [FindPackageShare("nesfr_arm_bringup"), "config", robot_config_file]
            )

    nesfr7_arm_node = Node(
        package='nesfr_arm_only_node_py',
        executable='nesfr_arm_only_node',
        namespace=namespace,
        name='nesfr7_arm_node',
        parameters=[nesfr7_arm_params],
        #parameters=[{"param0": 1, "param1": 2}],
        output='both',
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
        robot_state_publisher_node,
        nesfr7_arm_node,
        joy_node,
        # TODO
#        nesfr_system_main,
    ])
