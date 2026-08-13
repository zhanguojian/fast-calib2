from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os
import yaml


def _resolve_params(context):
    """加载 YAML，替换 $(find fast_calib)，并用 launch 参数覆盖非空项。"""
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

    overrides = {
        'bag_path': LaunchConfiguration('bag_path').perform(context),
        'image_path': LaunchConfiguration('image_path').perform(context),
        'lidar_topic': LaunchConfiguration('lidar_topic').perform(context),
        'lidar_forward_axis': LaunchConfiguration('lidar_forward_axis').perform(context),
        'lidar_up_axis': LaunchConfiguration('lidar_up_axis').perform(context),
        'output_path': LaunchConfiguration('output_path').perform(context),
        'use_auto_lidar_roi': LaunchConfiguration('use_auto_lidar_roi').perform(context),
    }
    for key, value in overrides.items():
        if value is None or value == '':
            continue
        if key == 'use_auto_lidar_roi':
            params[key] = value.lower() in ('1', 'true', 'yes')
        else:
            params[key] = value

    if not params.get('output_path'):
        params['output_path'] = os.path.join(pkg_share, 'output')

    return pkg_share, params


def _launch_setup(context, *args, **kwargs):
    pkg_share, params = _resolve_params(context)
    nodes = [
        Node(
            package='fast_calib',
            executable='fast_calib_node',
            name='mono_qr_pattern',
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
                output='screen',
            )
        )
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('config_file', default_value=''),
        DeclareLaunchArgument('bag_path', default_value=''),
        DeclareLaunchArgument('image_path', default_value=''),
        DeclareLaunchArgument('lidar_topic', default_value=''),
        DeclareLaunchArgument('lidar_forward_axis', default_value=''),
        DeclareLaunchArgument('lidar_up_axis', default_value=''),
        DeclareLaunchArgument('output_path', default_value=''),
        DeclareLaunchArgument('use_auto_lidar_roi', default_value=''),
        OpaqueFunction(function=_launch_setup),
    ])
