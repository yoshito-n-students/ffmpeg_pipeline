from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [
                    FindPackageShare('ffmpeg_pipeline_examples'),
                    'pulse_audio',
                    'capture',
                    'robot_description.urdf.xacro',
                ]
            ),
        ]
    )
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('ffmpeg_pipeline_examples'),
            'pulse_audio',
            'capture',
            'controllers.yaml',
        ]
    )

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_controllers],
        output='both',
    )

    robot_description_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )

    packet_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            '--param-file',
            robot_controllers,
            '--activate-as-group',
            'raw_packet_broadcaster',
            'decoder_filter',
            'frame_broadcaster',
            'encoder_filter',
            'packet_broadcaster',
        ],
    )

    nodes = [
        control_node,
        robot_description_pub_node,
        packet_broadcaster_spawner,
    ]

    return LaunchDescription(nodes)
