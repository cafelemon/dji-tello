from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    share = FindPackageShare('tello_bringup')
    default_config = PathJoinSubstitution([share, 'config', 'default.yaml'])
    jetson_config = PathJoinSubstitution([share, 'config', 'jetson.yaml'])
    parameters = [default_config, jetson_config]
    return LaunchDescription([
        Node(
            package='tello_transport_cpp',
            executable='tello_transport_node',
            name='tello_transport',
            parameters=parameters,
            output='screen',
        ),
        Node(
            package='tello_flight_manager',
            executable='flight_manager',
            name='tello_flight_manager',
            parameters=parameters,
            output='screen',
        ),
        Node(
            package='tello_vision',
            executable='tracker_node',
            name='tello_tracker',
            parameters=parameters,
            output='screen',
        ),
        Node(
            package='tello_system_monitor',
            executable='system_monitor',
            name='tello_system_monitor',
            parameters=parameters,
            output='screen',
        ),
    ])
