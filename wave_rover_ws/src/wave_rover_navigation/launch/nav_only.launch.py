"""
nav_only.launch.py  –  Nav2 + AMCL + map_server + EKF only (no SLAM)

Started dynamically by node_manager_node when nav_mode = "nav".
Requires a pre-built map (pass map:=/path/to/map.yaml).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav_pkg  = get_package_share_directory('wave_rover_navigation')
    nav2_pkg = get_package_share_directory('nav2_bringup')

    map_arg  = DeclareLaunchArgument('map',          default_value='')
    sim_arg  = DeclareLaunchArgument('use_sim_time', default_value='false')

    map_path     = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')

    nav2_params = os.path.join(nav_pkg, 'config', 'nav2_params.yaml')
    ekf_params  = os.path.join(nav_pkg, 'config', 'ekf_params.yaml')

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params, {'use_sim_time': use_sim_time}],
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_pkg, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file':  nav2_params,
            'map':          map_path,
        }.items(),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_pkg, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time':    use_sim_time,
            'params_file':     nav2_params,
            'use_composition': 'false',
        }.items(),
    )

    return LaunchDescription([map_arg, sim_arg, ekf_node, localization, navigation])
