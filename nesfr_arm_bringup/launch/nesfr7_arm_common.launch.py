from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (EnvironmentVariable, FindExecutable,
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
        output='both',
    )
    return LaunchDescription([
        namespace_launch_arg,
        nesfr_arm_node,
    ])
