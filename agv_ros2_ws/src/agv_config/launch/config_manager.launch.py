from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_path_arg = DeclareLaunchArgument(
        'config_path',
        default_value='/workspace/agv_ros2_ws/src/agv_config/config/default_config.yaml',
        description='Path to config file'
    )

    config_manager_node = Node(
        package='agv_config',
        executable='config_manager_node',
        name='config_manager',
        output='screen',
        parameters=[
            {'config_path': LaunchConfiguration('config_path')},
        ],
    )

    return LaunchDescription([
        config_path_arg,
        config_manager_node,
    ])
