from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node, PushRosNamespace
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ##################
    # Launch Arguments
    ##################

    # namespace
    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for the nodes, topics, and parameters'
    )
    namespace = LaunchConfiguration('namespace')

    #######
    # Nodes
    #######

    # Publisher for the hardware_description topic (required by the ros2_control_node)
    hw_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [
                    FindPackageShare('ffmpeg_pipeline_examples'),
                    'wav_file',
                    'hardware_description.urdf.xacro',
                ]
            ),
        ]
    )
    hw_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='hardware_description_publisher',
        parameters=[{'robot_description': ParameterValue(hw_description_content, value_type=str)}],
        remappings=[('robot_description', 'hardware_description')],
        output='both',
    )

    # The ros2_control_node
    ffmpeg_controllers = PathJoinSubstitution(
        [
            FindPackageShare('ffmpeg_pipeline_examples'),
            'wav_file',
            'controllers.yaml',
        ]
    )
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='ros2_control_node',
        parameters=[ffmpeg_controllers],
        remappings=[('robot_description', 'hardware_description')],
        output='both',
    )

    # Controller spawner
    controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='controller_spawner',
        arguments=[
            '--controller-manager',
            'ros2_control_node',
            '--param-file',
            ffmpeg_controllers,
            '--activate-as-group',
            'raw_packet_broadcaster',
            'decoder_filter',
            'frame_broadcaster',
            'sample_rate_converter',
            'frame_size_converter',
            'encoder_filter',
            'packet_broadcaster',
        ],
        output='both',
    )

    return LaunchDescription(
        [
            declare_namespace,
            GroupAction(
                [
                    PushRosNamespace(namespace),
                    hw_state_publisher_node,
                    ros2_control_node,
                    controller_spawner,
                ]
            ),
        ]
    )