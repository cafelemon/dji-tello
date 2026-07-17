from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    config = PathJoinSubstitution([FindPackageShare('tello_bringup'), 'config', 'default.yaml'])
    return LaunchDescription([
        Node(
            package='tello_mock',
            executable='flight_fault_harness',
            name='flight_fault_harness',
            output='screen',
        ),
        Node(
            package='tello_flight_manager',
            executable='flight_manager',
            name='tello_flight_manager',
            parameters=[config, {
                'link_status_stale_s': 0.3,
                'tracking_status_stale_s': 0.3,
                'cmd_stale_s': 0.2,
            }],
            output='screen',
        ),
    ])
