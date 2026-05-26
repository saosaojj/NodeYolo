from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='8080',
        description='Web server port')

    host_arg = DeclareLaunchArgument(
        'host',
        default_value='0.0.0.0',
        description='Web server host')

    static_dir_arg = DeclareLaunchArgument(
        'static_dir',
        default_value=PathJoinSubstitution([
            FindPackageShare('agv_web_frontend'),
        ]),
        description='Path to static frontend files')

    web_server_node = Node(
        package='agv_web_server',
        executable='web_server_node',
        name='web_server_node',
        output='screen',
        parameters=[{
            'host': LaunchConfiguration('host'),
            'port': LaunchConfiguration('port'),
            'static_dir': LaunchConfiguration('static_dir'),
        }],
    )

    return LaunchDescription([
        port_arg,
        host_arg,
        static_dir_arg,
        web_server_node,
    ])
