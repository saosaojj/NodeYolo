from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='agv_utils',
            executable='performance_monitor',
            name='performance_monitor',
            output='screen',
            parameters=[{
                'monitor_rate': 1.0,
                'alert_threshold_ms': 100.0,
            }]
        ),
    ])
