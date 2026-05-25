from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    map_width_arg = DeclareLaunchArgument(
        'map_width',
        default_value='20.0',
        description='Map width in meters'
    )
    
    map_height_arg = DeclareLaunchArgument(
        'map_height',
        default_value='20.0',
        description='Map height in meters'
    )
    
    resolution_arg = DeclareLaunchArgument(
        'resolution',
        default_value='0.05',
        description='Map resolution in meters'
    )
    
    config_file = LaunchConfiguration('config_file', default=[
        '/workspace/agv_ros2_ws/src/agv_path_planner/config/planner_config.yaml'
    ])
    
    path_planner_node = Node(
        package='agv_path_planner',
        executable='path_planner_node',
        name='path_planner_node',
        output='screen',
        parameters=[{
            'map_width': LaunchConfiguration('map_width'),
            'map_height': LaunchConfiguration('map_height'),
            'map_resolution': LaunchConfiguration('resolution'),
        }]
    )
    
    return LaunchDescription([
        map_width_arg,
        map_height_arg,
        resolution_arg,
        path_planner_node
    ])
