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

    # Publisher node for the hardware_description topic.
    # It passes the contents of hardware_description.urdf.xacro as a message to ros2_control_node.
    hw_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [
                    FindPackageShare('ffmpeg_pipeline_examples'),
                    'h264_camera',
                    'hardware_description.urdf.xacro',
                ]
            ),
        ]
    )
    hw_description_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='hardware_description_publisher',
        output='both',
        parameters=[{'robot_description': ParameterValue(hw_description_content, value_type=str)}],
        remappings=[('robot_description', 'hardware_description')],
    )

    # The ros2_control_node.
    # It loads the hardware plugin described in the hardware_description message
    # and controller plugins asked from the controller spawner,
    # and drives the hardware and controllers.
    ffmpeg_controllers = PathJoinSubstitution(
        [
            FindPackageShare('ffmpeg_pipeline_examples'),
            'h264_camera',
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

    # The controller spawner which asks the ros2_control_node to load the controllers
    controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='controller_spawner',
        arguments=[
            '--param-file',
            ffmpeg_controllers,
            'compressed_image_broadcaster',
            ],
    )

    ####################
    # Launch Description
    ####################

    # Declare the launch arguments and the nodes pushed into the namespace
    return LaunchDescription(
        [
            declare_namespace,
            GroupAction(
                [
                    PushRosNamespace(namespace),
                    hw_description_publisher,
                    ros2_control_node,
                    controller_spawner,
                ]
            ),
        ]
    )
