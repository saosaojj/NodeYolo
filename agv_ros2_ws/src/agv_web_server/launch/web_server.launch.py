from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
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

    web_server_node = Node(
        package='agv_web_server',
        executable='web_server_node',
        name='web_server_node',
        output='screen',
        parameters=[{
            'host': LaunchConfiguration('host'),
            'port': LaunchConfiguration('port'),
        }],
    )

    return LaunchDescription([
        port_arg,
        host_arg,
        web_server_node,
    ])
