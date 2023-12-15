import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('raros'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='raros',
            namespace='raros',
            executable='status',
            name='status'
        ),
        Node(
            package='raros',
            namespace='raros',
            executable='action_api',
            name='action_api'
        ),
        Node(
            package='raros',
            namespace='raros',
            executable='magnet',
            name='magnet',
            parameters=[config]
        ),
        Node(
            package='raros',
            namespace='raros',
            executable='buzzer',
            name='buzzer'
        ),
        Node(
            package='raros',
            namespace='raros',
            executable='color_sensor',
            name='color_sensor'
        ),
        Node(
            package='raros',
            namespace='raros',
            executable='range_sensor',
            name='range_sensor'
        ),
        Node(
            package='raros',
            namespace='raros',
            executable='navigation',
            name='navigation'
        ),
        Node(
            package='raros',
            namespace='raros',
            executable='collision_detection',
            name='collision_detection'
        ),
        Node(
            package='raros',
            namespace='raros',
            executable='camera',
            name='camera'
        ),
    ])
