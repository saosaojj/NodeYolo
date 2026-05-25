import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_simulator_arg = DeclareLaunchArgument(
        'use_simulator',
        default_value='False',
        description='Whether to start the PLC simulator for testing',
    )

    config_file_default = os.path.join(
        get_package_share_directory('agv_plc_bridge'),
        'config',
        'plc_config.yaml',
    )

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=config_file_default,
        description='Path to the PLC configuration YAML file',
    )

    simulator_node = Node(
        package='agv_plc_bridge',
        executable='plc_simulator',
        name='plc_simulator',
        output='screen',
        parameters=[{'port': 5020}],
        condition=IfCondition(LaunchConfiguration('use_simulator')),
    )

    manager_node = Node(
        package='agv_plc_bridge',
        executable='plc_manager_node',
        name='plc_manager_node',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
    )

    return LaunchDescription([
        use_simulator_arg,
        config_file_arg,
        simulator_node,
        manager_node,
    ])
