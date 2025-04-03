from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("ffmpeg_pipeline_examples"),
                    "h264_stream",
                    "robot_description.urdf",
                ]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("ffmpeg_pipeline_examples"),
            "h264_stream",
            "controllers.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
    )

    robot_description_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    packet_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ffmpeg_packet_broadcaster", "--param-file", robot_controllers],
    )

    nodes = [
        control_node,
        robot_description_pub_node,
        packet_broadcaster_spawner,
    ]

    return LaunchDescription(nodes)