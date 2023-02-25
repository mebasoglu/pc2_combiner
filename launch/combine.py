from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['--x', '0.0', '--y', '0.56362', '--z', '-0.30555', '--yaw', '1.575', '--pitch', '0.71',
                       '--roll', '-0.02', '--frame-id', 'base_link', '--child-frame-id', 'velodyne_left']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['--x', '0.0', '--y', '0.56362', '--z', '-0.30555', '--yaw', '-1.580', '--pitch', '0.71',
                       '--roll', '-0.01', '--frame-id', 'base_link', '--child-frame-id', 'velodyne_right']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['--x', '0.0', '--y', '0.0', '--z', '0.0', '--yaw', '1.575', '--pitch', '0.0',
                       '--roll', '0.0', '--frame-id', 'base_link', '--child-frame-id', 'velodyne_top']
        ),

    ])
