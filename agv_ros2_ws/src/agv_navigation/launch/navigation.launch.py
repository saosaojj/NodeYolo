import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('agv_navigation')
    config_file = os.path.join(pkg_share, 'config', 'navigation_config.yaml')

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='agv',
        description='Namespace for the AGV navigation nodes')

    namespace = LaunchConfiguration('namespace')

    agv_odometry_node = Node(
        package='agv_navigation',
        executable='agv_odometry_node',
        name='agv_odometry',
        namespace=namespace,
        parameters=[config_file],
        output='screen')

    agv_controller_node = Node(
        package='agv_navigation',
        executable='agv_controller_node',
        name='agv_controller',
        namespace=namespace,
        parameters=[config_file],
        output='screen')

    agv_navigator_node = Node(
        package='agv_navigation',
        executable='agv_navigator_node',
        name='agv_navigator',
        namespace=namespace,
        parameters=[config_file],
        output='screen')

    return LaunchDescription([
        namespace_arg,
        agv_odometry_node,
        agv_controller_node,
        agv_navigator_node,
    ])
