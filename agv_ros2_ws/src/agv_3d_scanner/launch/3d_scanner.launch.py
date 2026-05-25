from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    scan_resolution_arg = DeclareLaunchArgument(
        'scan_resolution',
        default_value='0.05',
        description='Scan resolution in meters'
    )

    export_path_arg = DeclareLaunchArgument(
        'export_path',
        default_value='/tmp/agv_maps',
        description='Default export path for maps'
    )

    export_format_arg = DeclareLaunchArgument(
        'export_format',
        default_value='pcd',
        description='Default export format (xyz, pcd, ply)'
    )

    scanner_node = Node(
        package='agv_3d_scanner',
        executable='scanner_node',
        name='scanner_node',
        output='screen',
        parameters=[
            '/workspace/agv_ros2_ws/src/agv_3d_scanner/config/scanner_config.yaml',
            {
                'scan_resolution': LaunchConfiguration('scan_resolution'),
            },
        ],
    )

    point_cloud_manager_node = Node(
        package='agv_3d_scanner',
        executable='point_cloud_manager',
        name='point_cloud_manager',
        output='screen',
        parameters=[
            '/workspace/agv_ros2_ws/src/agv_3d_scanner/config/scanner_config.yaml',
        ],
    )

    map_exporter_node = Node(
        package='agv_3d_scanner',
        executable='map_exporter',
        name='map_exporter',
        output='screen',
        parameters=[
            '/workspace/agv_ros2_ws/src/agv_3d_scanner/config/scanner_config.yaml',
            {
                'default_export_path': LaunchConfiguration('export_path'),
                'default_format': LaunchConfiguration('export_format'),
            },
        ],
    )

    return LaunchDescription([
        scan_resolution_arg,
        export_path_arg,
        export_format_arg,
        scanner_node,
        point_cloud_manager_node,
        map_exporter_node,
    ])
