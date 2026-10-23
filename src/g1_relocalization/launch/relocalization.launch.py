import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory("g1_relocalization"),
        "config",
        "relocalization.yaml",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_path', default_value=os.environ.get(
                'G1_MAP_PCD', os.path.join(os.environ['G1_WS'], 'maps', 'map.pcd')
                if os.environ.get('G1_WS') else ''),
            description='PC2 PCD map path (or export G1_MAP_PCD / G1_WS)',
        ),
        Node(
            package="g1_relocalization",
            executable="g1_relocalization_node",
            name="g1_relocalization",
            output="screen",
            parameters=[config_file, {'map_path': LaunchConfiguration('map_path')}],
        )
    ])
