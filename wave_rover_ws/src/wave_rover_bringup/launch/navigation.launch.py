"""
navigation.launch.py  –  Nav2 + SLAM/localization for Wave Rover

Must be run after (or together with) bringup.launch.py.

Usage:
  # Mapping: build a new map with SLAM
  ros2 launch wave_rover_bringup navigation.launch.py mode:=slam

  # Navigation: use a previously saved map
  ros2 launch wave_rover_bringup navigation.launch.py mode:=nav map:=/path/to/map.yaml

  # Save map when done (SLAM mode):
  ros2 run nav2_map_server map_saver_cli -f ~/my_map

Arguments:
  mode          slam | nav       default: slam
  map           path to .yaml    required in nav mode
  use_rviz      true | false     default: true
  use_sim_time  true | false     default: false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav_pkg    = get_package_share_directory('wave_rover_navigation')
    bringup_pkg = get_package_share_directory('wave_rover_bringup')

    # ── arguments ─────────────────────────────────────────────────────────
    args = [
        DeclareLaunchArgument('mode',         default_value='slam',
                              description='"slam" for mapping or "nav" for navigation'),
        DeclareLaunchArgument('map',          default_value='',
                              description='Map YAML path (nav mode only)'),
        DeclareLaunchArgument('use_rviz',     default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
    ]

    mode         = LaunchConfiguration('mode')
    map_path     = LaunchConfiguration('map')
    use_rviz     = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # ── nav stack ─────────────────────────────────────────────────────────
    nav_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_pkg, 'launch', 'nav_stack.launch.py')
        ),
        launch_arguments={
            'mode':         mode,
            'map':          map_path,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    # ── RViz ─────────────────────────────────────────────────────────────
    rviz_config = os.path.join(bringup_pkg, 'config', 'wave_rover_nav.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        *args,
        LogInfo(msg=['Navigation launch: mode=', mode, '  map=', map_path]),
        nav_stack,
        rviz_node,
    ])
