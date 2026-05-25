from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    simulate_arg = DeclareLaunchArgument(
        'simulate',
        default_value='true',
        description='Enable simulated battery data'
    )

    auto_charge_arg = DeclareLaunchArgument(
        'auto_charge',
        default_value='true',
        description='Enable automatic charging when battery is low'
    )

    battery_monitor_node = Node(
        package='agv_power_manager',
        executable='battery_monitor_node',
        name='battery_monitor',
        output='screen',
        parameters=[
            {'simulate': LaunchConfiguration('simulate')},
        ],
    )

    charging_controller_node = Node(
        package='agv_power_manager',
        executable='charging_controller_node',
        name='charging_controller',
        output='screen',
        parameters=[
            {'auto_charge_enabled': LaunchConfiguration('auto_charge')},
        ],
    )

    power_manager_node = Node(
        package='agv_power_manager',
        executable='power_manager_node',
        name='power_manager',
        output='screen',
    )

    return LaunchDescription([
        simulate_arg,
        auto_charge_arg,
        battery_monitor_node,
        charging_controller_node,
        power_manager_node,
    ])
