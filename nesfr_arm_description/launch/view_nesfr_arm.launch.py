#
# Author: Jaeyoung Lee

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import TimerAction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # General arguments
    description_package = LaunchConfiguration("description_package")
    description_package_arg = DeclareLaunchArgument(
        "description_package",
        default_value="nesfr_arm_description",
        description="Description package with robot URDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom description.",
        )

    description_file = LaunchConfiguration("description_file")
    description_file_arg = DeclareLaunchArgument(
        "description_file",
        default_value="nesfr_arm.urdf.xacro",
        description="URDF/XACRO description file with the robot.",
        )

    robot_name = LaunchConfiguration("robot_name")
    robot_name_arg = DeclareLaunchArgument(
        "robot_name",
        default_value="nesfr7_arm",
        description="robot parameters",
        )

    # Initialize Arguments

    # General arguments
    nesfr_arm_params_file = LaunchConfiguration('nesfr_arm_params_file', default=[robot_name, '.yaml'])

    #
    # reference: https://answers.ros.org/question/401950/ros2-any-idea-on-how-to-pass-parameters-yaml-to-a-xacro-file-via-launch-python-script/
    #
    nesfr_arm_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", nesfr_arm_params_file]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
            " ",
            "nesfr_arm_params:=",
            nesfr_arm_params,
            " ",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", "view_robot.rviz"]
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen",
    )
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="both",
        arguments=["-d", rviz_config_file],
    )

    nodes_to_start = [
        robot_state_publisher_node,
        joint_state_publisher_node,
        # https://answers.ros.org/question/387310/run-ros2-node-using-launch-file-with-delay/
        # one second delay after setup
        TimerAction(period=1.0, actions=[rviz_node,]),
    ]

    return LaunchDescription([
        description_package_arg,
        description_file_arg,
        robot_name_arg,
    ] + nodes_to_start)
