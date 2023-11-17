from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='raros',
            namespace='raros',
            executable='magnet',
            name='magnet'
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
            executable='action_api',
            name='action_api'
        ),
    ])
