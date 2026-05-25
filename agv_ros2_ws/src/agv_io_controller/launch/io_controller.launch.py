import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    simulate_arg = DeclareLaunchArgument(
        'simulate',
        default_value='True',
        description='Whether to run in simulation mode',
    )

    config_file_default = os.path.join(
        get_package_share_directory('agv_io_controller'),
        'config',
        'io_config.yaml',
    )

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=config_file_default,
        description='Path to the IO configuration YAML file',
    )

    simulator_node = Node(
        package='agv_io_controller',
        executable='io_simulator_node',
        name='io_simulator_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('simulate')),
    )

    controller_node = Node(
        package='agv_io_controller',
        executable='io_controller_node',
        name='io_controller_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {'simulate': LaunchConfiguration('simulate')},
        ],
    )

    return LaunchDescription([
        simulate_arg,
        config_file_arg,
        simulator_node,
        controller_node,
    ])
