from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    camera_id_arg = DeclareLaunchArgument(
        'camera_id',
        default_value='main_camera',
        description='Camera identifier'
    )

    device_path_arg = DeclareLaunchArgument(
        'device_path',
        default_value='/dev/video0',
        description='Camera device path'
    )

    width_arg = DeclareLaunchArgument(
        'width',
        default_value='640',
        description='Frame width'
    )

    height_arg = DeclareLaunchArgument(
        'height',
        default_value='480',
        description='Frame height'
    )

    fps_arg = DeclareLaunchArgument(
        'fps',
        default_value='30',
        description='Frames per second'
    )

    use_rtsp_arg = DeclareLaunchArgument(
        'use_rtsp',
        default_value='False',
        description='Use RTSP stream'
    )

    rtsp_url_arg = DeclareLaunchArgument(
        'rtsp_url',
        default_value='',
        description='RTSP stream URL'
    )

    camera_manager_node = Node(
        package='agv_camera_manager',
        executable='camera_manager_node',
        name='camera_manager',
        output='screen',
        parameters=[
            '/workspace/agv_ros2_ws/src/agv_camera_manager/config/camera_config.yaml',
            {
                'camera_id': LaunchConfiguration('camera_id'),
                'device_path': LaunchConfiguration('device_path'),
                'width': LaunchConfiguration('width'),
                'height': LaunchConfiguration('height'),
                'fps': LaunchConfiguration('fps'),
                'use_rtsp': LaunchConfiguration('use_rtsp'),
                'rtsp_url': LaunchConfiguration('rtsp_url'),
            },
        ],
    )

    return LaunchDescription([
        camera_id_arg,
        device_path_arg,
        width_arg,
        height_arg,
        fps_arg,
        use_rtsp_arg,
        rtsp_url_arg,
        camera_manager_node,
    ])
