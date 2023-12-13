#
# Author: Jaeyoung Lee

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import TimerAction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]


    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare('nesfr_arm_bringup'), "rviz", "view_robot.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        namespace=LaunchConfiguration('namespace'),
        output="both",
        arguments=["-d", rviz_config_file],
        remappings=remappings,
    )

    nodes_to_start = [
        rviz_node,
    ]

    return LaunchDescription(nodes_to_start)
