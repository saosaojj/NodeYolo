import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config_file_default = os.path.join(
        get_package_share_directory('agv_connectivity'),
        'config',
        'connectivity_config.yaml',
    )

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=config_file_default,
        description='Path to the connectivity configuration YAML file',
    )

    wifi_manager_node = Node(
        package='agv_connectivity',
        executable='wifi_manager_node',
        name='wifi_manager_node',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
    )

    bluetooth_manager_node = Node(
        package='agv_connectivity',
        executable='bluetooth_manager_node',
        name='bluetooth_manager_node',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
    )

    return LaunchDescription([
        config_file_arg,
        wifi_manager_node,
        bluetooth_manager_node,
    ])
