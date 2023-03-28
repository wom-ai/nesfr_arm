from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            LogInfo, RegisterEventHandler, TimerAction)

from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)
from launch.events import Shutdown

from launch.substitutions import (Command, EnvironmentVariable, FindExecutable,
                                LaunchConfiguration, LocalSubstitution,
                                PythonExpression, PathJoinSubstitution)

def generate_launch_description():
    #
    # references
    #  - https://answers.ros.org/question/384712/ros2-launch-how-to-concatenate-launchconfiguration-with-string/
    #  - https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/foxy/ur_bringup/launch/ur_control.launch.py#L219
    #
    namespace = LaunchConfiguration('namespace')
    namespace_launch_arg = DeclareLaunchArgument(
            'namespace',
            default_value='please_set_namespace'
            )
    #
    # nesfr_arm_node
    #
    robot_config_file = LaunchConfiguration('robot_config_file', default=[namespace, '.yaml'])
    nesfr7_arm_params = PathJoinSubstitution(
            [FindPackageShare("nesfr_arm_bringup"), "config", robot_config_file]
            )

    nesfr_arm_node = Node(
        package='nesfr_arm_node',
        executable='nesfr_arm_node',
        namespace=namespace,
        name='nesfr7_arm_node',
        parameters=[nesfr7_arm_params],
        #parameters=[{"param0": 1, "param1": 2}],
        arguments=['--ros-args', '--log-level', [namespace, '.nesfr7_arm_node:=info'],],
        output='both',
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
            namespace, '/',
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


    return LaunchDescription([
        namespace_launch_arg,
        robot_state_publisher_node,
        nesfr_arm_node,
        RegisterEventHandler(
            OnProcessExit(
                target_action=nesfr_arm_node,
                on_exit=[
                    LogInfo(msg=(' closed nesfr_arm_node')),
                    EmitEvent(event=Shutdown(reason='Window closed'))
                    ]
                )
            ),
    ])
