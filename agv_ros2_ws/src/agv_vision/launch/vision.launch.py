from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_camera_arg = DeclareLaunchArgument(
        'use_camera',
        default_value='True',
        description='Whether to launch the camera node'
    )

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

    camera_node = Node(
        package='agv_vision',
        executable='camera_node',
        name='camera',
        output='screen',
        parameters=[
            '/workspace/agv_ros2_ws/src/agv_vision/config/vision_config.yaml',
        ],
        condition=IfCondition(LaunchConfiguration('use_camera')),
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
        use_camera_arg,
        model_path_arg,
        device_arg,
        camera_node,
        yolo_detector_node,
    ])
