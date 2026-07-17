from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    config = PathJoinSubstitution([FindPackageShare('tello_bringup'), 'config', 'default.yaml'])
    video_path = LaunchConfiguration('video_path')
    auto_select_target = LaunchConfiguration('auto_select_target')
    publish_rate_hz = LaunchConfiguration('publish_rate_hz')
    return LaunchDescription([
        DeclareLaunchArgument('video_path', description='Video file used for offline replay'),
        DeclareLaunchArgument('auto_select_target', default_value='false'),
        DeclareLaunchArgument('publish_rate_hz', default_value='0.0'),
        Node(
            package='tello_vision',
            executable='offline_video_publisher',
            name='offline_video_publisher',
            parameters=[config, {'video_path': video_path, 'publish_rate_hz': publish_rate_hz}],
            output='screen',
        ),
        Node(
            package='tello_vision',
            executable='tracker_node',
            name='tello_tracker',
            parameters=[config, {'auto_select_target': auto_select_target}],
            output='screen',
        ),
    ])
