from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():
    use_recording_arg = DeclareLaunchArgument(
        'use_recording',
        default_value='True',
        description='Enable video recording'
    )
    
    use_server_arg = DeclareLaunchArgument(
        'use_server',
        default_value='True',
        description='Enable video server'
    )
    
    use_recording = LaunchConfiguration('use_recording')
    use_server = LaunchConfiguration('use_server')
    
    video_server_node = Node(
        package='agv_video_stream',
        executable='video_server_node',
        name='video_server_node',
        output='screen',
        condition=IfCondition(use_server),
        parameters=[{
            'port': 8554,
            'stream_quality': 85,
            'frame_rate': 30,
            'resolution_width': 640,
            'resolution_height': 480,
            'rtsp_path': 'stream'
        }]
    )
    
    video_recorder_node = Node(
        package='agv_video_stream',
        executable='video_recorder_node',
        name='video_recorder_node',
        output='screen',
        condition=IfCondition(use_recording),
        parameters=[{
            'record_path': '/tmp/agv_recordings',
            'max_storage_gb': 10.0,
            'min_free_space_gb': 1.0,
            'detection_threshold': 0.7,
            'record_duration_sec': 5.0,
            'snapshot_interval_sec': 0.5,
            'trigger_classes': ['person', 'obstacle', 'forklift', 'pallet']
        }]
    )
    
    return LaunchDescription([
        use_recording_arg,
        use_server_arg,
        video_server_node,
        video_recorder_node
    ])
