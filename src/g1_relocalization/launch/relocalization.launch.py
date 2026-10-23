import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory("g1_relocalization"),
        "config",
        "relocalization.yaml",
    )

    def node(context):
        # Convert explicitly: Foxy must receive a double even when the caller
        # writes initialization_timeout:=180 (without a decimal point).
        return [Node(
            package='g1_relocalization', executable='g1_relocalization_node',
            name='g1_relocalization', output='screen',
            parameters=[config_file, {
                'map_path': LaunchConfiguration('map_path').perform(context),
                'initialization_mode': LaunchConfiguration('initialization_mode').perform(context),
                'global_initialization_timeout_sec': float(
                    LaunchConfiguration('initialization_timeout').perform(context)),
                'diagnostics_directory': LaunchConfiguration('diagnostics_directory').perform(context),
            }],
        )]

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_path', default_value=os.environ.get(
                'G1_MAP_PCD', os.path.join(os.environ['G1_WS'], 'maps', 'map.pcd')
                if os.environ.get('G1_WS') else ''),
            description='PC2 PCD map path (or export G1_MAP_PCD / G1_WS)',
        ),
        DeclareLaunchArgument('initialization_mode', default_value='manual',
                              description='manual or global'),
        DeclareLaunchArgument('initialization_timeout',
                              default_value=os.environ.get('G1_LOCALIZATION_TIMEOUT_SEC', '180.0')),
        DeclareLaunchArgument('diagnostics_directory', default_value='',
                              description='Session evidence directory; defaults to workspace runtime'),
        OpaqueFunction(function=node),
    ])
