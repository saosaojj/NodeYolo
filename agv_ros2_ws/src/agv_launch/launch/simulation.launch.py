import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='agv',
        description='Namespace for all AGV nodes')

    web_port_arg = DeclareLaunchArgument(
        'web_port',
        default_value='8080',
        description='Web server port')

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('agv_navigation'),
                'launch',
                'navigation.launch.py')),
        launch_arguments={
            'namespace': LaunchConfiguration('namespace'),
        }.items())

    io_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('agv_io_controller'),
                'launch',
                'io_controller.launch.py')),
        launch_arguments={
            'simulate': 'True',
        }.items())

    plc_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('agv_plc_bridge'),
                'launch',
                'plc_bridge.launch.py')),
        launch_arguments={
            'use_simulator': 'True',
        }.items())

    vision_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('agv_vision'),
                'launch',
                'vision.launch.py')),
        launch_arguments={
            'use_camera': 'False',
        }.items())

    connectivity_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('agv_connectivity'),
                'launch',
                'connectivity.launch.py')))

    web_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('agv_web_server'),
                'launch',
                'web_server.launch.py')),
        launch_arguments={
            'port': LaunchConfiguration('web_port'),
        }.items())

    return LaunchDescription([
        namespace_arg,
        web_port_arg,
        navigation_launch,
        io_controller_launch,
        plc_bridge_launch,
        vision_launch,
        connectivity_launch,
        web_server_launch,
    ])
