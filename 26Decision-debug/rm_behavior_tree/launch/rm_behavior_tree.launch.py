import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    bt_config_dir = os.path.join(get_package_share_directory('rm_behavior_tree'), 'config')

    # Keep the generic launcher on the established ordinary strategy.  The
    # tunnel workflow explicitly supplies tunnel.xml via run_tunnel.sh.
    style = LaunchConfiguration('style', default='v2.xml')
    use_sim_time = LaunchConfiguration('use_sim_time', default='False')
    semantic_config = os.path.join(bt_config_dir, 'RMUL2027_semantic_map.yaml')

    bt_xml_dir = PathJoinSubstitution([bt_config_dir, style])

    rm_behavior_tree_node = Node(
        package='rm_behavior_tree',
        executable='rm_behavior_tree',
        # Keep failures visible during tunnel debugging. Automatic respawn can
        # hide the first exception and repeatedly collide with Groot's port.
        respawn=False,
        respawn_delay=3,
        parameters=[
            {
              'style': bt_xml_dir,
              'use_sim_time': use_sim_time,
              'semantic_config': semantic_config,
            }
        ]
    )

    return LaunchDescription([rm_behavior_tree_node])
