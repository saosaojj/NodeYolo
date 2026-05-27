import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('agv_motor_controller')
    config_file = os.path.join(pkg_share, 'config', 'motor_config.yaml')

    simulate_arg = DeclareLaunchArgument(
        'simulate',
        default_value='false',
        description='启用仿真模式，不连接真实硬件')

    simulate = LaunchConfiguration('simulate')

    motor_controller_node = Node(
        package='agv_motor_controller',
        executable='motor_controller_node',
        name='motor_controller',
        parameters=[config_file, {'simulate': simulate}],
        output='screen')

    return LaunchDescription([
        simulate_arg,
        motor_controller_node,
    ])
