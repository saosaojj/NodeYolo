from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    yolo_trainer_node = Node(
        package='agv_vision',
        executable='yolo_trainer_node',
        name='yolo_trainer',
        output='screen',
        parameters=[
            '/workspace/agv_ros2_ws/src/agv_vision/config/vision_config.yaml',
        ],
    )

    return LaunchDescription([
        yolo_trainer_node,
    ])
