from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os
import yaml


def _launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('fast_calib')
    config_file = LaunchConfiguration('config_file').perform(context)
    if not config_file:
        config_file = os.path.join(pkg_share, 'config', 'qr_params.yaml')

    with open(config_file, 'r', encoding='utf-8') as f:
        params = yaml.safe_load(f) or {}

    def resolve_find(value):
        if isinstance(value, str):
            return value.replace('$(find fast_calib)', pkg_share)
        return value

    for key, value in list(params.items()):
        params[key] = resolve_find(value)

    output_path = LaunchConfiguration('output_path').perform(context)
    if output_path:
        params['output_path'] = output_path
    if not params.get('output_path'):
        params['output_path'] = os.path.join(pkg_share, 'output')

    nodes = [
        Node(
            package='fast_calib',
            executable='multi_fast_calib',
            name='multi_fast_calib',
            output='screen',
            parameters=[params],
        )
    ]

    if LaunchConfiguration('rviz').perform(context).lower() in ('1', 'true'):
        rviz_cfg = os.path.join(pkg_share, 'rviz_cfg', 'fast_livo2.rviz')
        nodes.append(
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz',
                arguments=['-d', rviz_cfg],
                output='screen',
            )
        )
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('config_file', default_value=''),
        DeclareLaunchArgument('output_path', default_value=''),
        OpaqueFunction(function=_launch_setup),
    ])
