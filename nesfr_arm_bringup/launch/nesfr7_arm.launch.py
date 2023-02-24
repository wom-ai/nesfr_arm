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
    namespace = LaunchConfiguration('namespace')

    namespace_launch_arg = DeclareLaunchArgument(
            'namespace',
            default_value='please_set_namespace'
            )

    nesfr7_arm_param = PathJoinSubstitution(
            [FindPackageShare("nesfr_arm_description"), "config", "nesfr7_arm.yaml"]
            )
    
    nesfr_arm_node = Node(
        package='nesfr_arm_node',
        executable='nesfr_arm_node',
        namespace=namespace,
        name='nesfr7_arm_node',
        parameters=[nesfr7_arm_param],
        #parameters=[{"param0": 1, "param1": 2}],
        output='both',
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
                ('joy', 'xbox_joy'),
                ('joy/set_feedback', 'xbox_joy/set_feedback'),
                ],
            output='both',
            parameters=[joy_params])
    return LaunchDescription([
        namespace_launch_arg,
        nesfr_arm_node,
        joy_node,
    ])
