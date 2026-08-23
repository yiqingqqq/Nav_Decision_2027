import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bt_config_dir = os.path.join(get_package_share_directory('rm_behavior_tree'), 'config')

    style = LaunchConfiguration('style')
    field_config = LaunchConfiguration('field_config')
    strategy_config = LaunchConfiguration('strategy_config')
    preset = LaunchConfiguration('preset')
    alliance = LaunchConfiguration('alliance')
    map_frame = LaunchConfiguration('map_frame')
    field_origin_x = LaunchConfiguration('field_origin_x')
    field_origin_y = LaunchConfiguration('field_origin_y')
    field_origin_yaw = LaunchConfiguration('field_origin_yaw')
    use_sim_time = LaunchConfiguration('use_sim_time')

    bt_xml_dir = PathJoinSubstitution([bt_config_dir, style])

    launch_arguments = [
        DeclareLaunchArgument('style', default_value='rmuc_2026.xml'),
        DeclareLaunchArgument(
            'field_config',
            default_value=os.path.join(bt_config_dir, 'rmuc_2026_field.yaml')),
        DeclareLaunchArgument(
            'strategy_config',
            default_value=os.path.join(bt_config_dir, 'rmuc_2026_strategy.yaml')),
        DeclareLaunchArgument('preset', default_value='balanced'),
        DeclareLaunchArgument('alliance', default_value='red'),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('field_origin_x', default_value='0.0'),
        DeclareLaunchArgument('field_origin_y', default_value='0.0'),
        DeclareLaunchArgument('field_origin_yaw', default_value='0.0'),
        DeclareLaunchArgument('use_sim_time', default_value='False'),
    ]

    rm_behavior_tree_node = Node(
        package='rm_behavior_tree',
        executable='rm_behavior_tree',
        respawn=True,
        respawn_delay=3,
        parameters=[
            field_config,
            strategy_config,
            {
                'style': bt_xml_dir,
                'strategy.active_preset': preset,
                'field_map.alliance': alliance,
                'field_map.frame_id': map_frame,
                'field_map.field_to_map.x': ParameterValue(field_origin_x, value_type=float),
                'field_map.field_to_map.y': ParameterValue(field_origin_y, value_type=float),
                'field_map.field_to_map.yaw': ParameterValue(field_origin_yaw, value_type=float),
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            }
        ]
    )

    return LaunchDescription(launch_arguments + [rm_behavior_tree_node])
