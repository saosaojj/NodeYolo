from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='yolov8n.pt',
        description='Path to the YOLO model file'
    )

    device_arg = DeclareLaunchArgument(
        'device',
        default_value='cpu',
        description='Device for inference (cpu or cuda)'
    )

    yolo_detector_node = Node(
        package='agv_vision',
        executable='yolo_detector_node',
        name='yolo_detector',
        output='screen',
        parameters=[
            '/workspace/agv_ros2_ws/src/agv_vision/config/vision_config.yaml',
            {
                'model_path': LaunchConfiguration('model_path'),
                'device': LaunchConfiguration('device'),
            },
        ],
    )

    return LaunchDescription([
        model_path_arg,
        device_arg,
        yolo_detector_node,
    ])
