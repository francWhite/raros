import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('raros'), 'config')
    params_config = os.path.join(config_dir, 'params.yaml')
    params_calibrated_config = os.path.join(config_dir, 'params_calibrated.yaml')

    parameters = [params_config]
    if os.path.isfile(params_calibrated_config):
        print('params_calibrated.yaml file has been found, some parameters will be overwritten')
        parameters.append(params_calibrated_config)

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
            parameters=parameters
        ),
        Node(
            package='raros',
            namespace='raros',
            executable='buzzer',
            name='buzzer',
            parameters=parameters
        ),
        Node(
            package='raros',
            namespace='raros',
            executable='color_sensor',
            name='color_sensor',
            parameters=parameters
        ),
        Node(
            package='raros',
            namespace='raros',
            executable='range_sensor',
            name='range_sensor',
            parameters=parameters
        ),
        Node(
            package='raros',
            namespace='raros',
            executable='navigation',
            name='navigation',
            parameters=parameters
        ),
        Node(
            package='raros',
            namespace='raros',
            executable='collision_detection',
            name='collision_detection',
            parameters=parameters
        ),
        Node(
            package='raros',
            namespace='raros',
            executable='camera',
            name='camera',
            parameters=parameters
        ),
    ])
