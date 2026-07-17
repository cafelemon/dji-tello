from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    config = PathJoinSubstitution([FindPackageShare('tello_bringup'), 'config', 'default.yaml'])
    return LaunchDescription([
        Node(
            package='tello_transport_cpp',
            executable='tello_transport_node',
            name='tello_transport',
            parameters=[config],
            output='screen',
        ),
        Node(
            package='tello_flight_manager',
            executable='flight_manager',
            name='tello_flight_manager',
            parameters=[config],
            output='screen',
        ),
        Node(
            package='tello_vision',
            executable='tracker_node',
            name='tello_tracker',
            parameters=[config],
            output='screen',
        ),
    ])
