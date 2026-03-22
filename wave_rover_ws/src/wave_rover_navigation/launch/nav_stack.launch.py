"""
nav_stack.launch.py  –  Nav2 + SLAM Toolbox + robot_localization EKF

Usage:
  # SLAM (mapping) mode
  ros2 launch wave_rover_navigation nav_stack.launch.py mode:=slam

  # Navigation mode (needs a pre-built map)
  ros2 launch wave_rover_navigation nav_stack.launch.py mode:=nav map:=/path/to/map.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                             IncludeLaunchDescription, LogInfo)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap


def get_share(pkg: str) -> str:
    return get_package_share_directory(pkg)


def generate_launch_description():
    nav_pkg  = get_share('wave_rover_navigation')
    nav2_pkg = get_share('nav2_bringup')

    # ── arguments ────────────────────────────────────────────────────────
    mode_arg = DeclareLaunchArgument(
        'mode', default_value='slam',
        description='"slam" = mapping mode | "nav" = localization+nav mode')
    map_arg  = DeclareLaunchArgument(
        'map', default_value='',
        description='Path to map YAML (only used in "nav" mode)')
    sim_arg  = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation clock')

    mode         = LaunchConfiguration('mode')
    map_path     = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')

    nav2_params  = os.path.join(nav_pkg, 'config', 'nav2_params.yaml')
    slam_params  = os.path.join(nav_pkg, 'config', 'slam_params.yaml')
    ekf_params   = os.path.join(nav_pkg, 'config', 'ekf_params.yaml')

    # ── robot_localization EKF (always on) ────────────────────────────────
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params, {'use_sim_time': use_sim_time}],
    )

    # ── SLAM Toolbox (mapping mode) ───────────────────────────────────────
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params, {'use_sim_time': use_sim_time}],
        condition=IfCondition(
            # Python-style equality check not available in launch – use a workaround
            # by always starting; slam_params mode=mapping handles it
            'true'
        ) if False else None,   # placeholder – see GroupAction below
    )

    # ── SLAM group (active only in slam mode) ─────────────────────────────
    slam_group = GroupAction(
        actions=[
            LogInfo(msg='Starting SLAM Toolbox in mapping mode'),
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[slam_params, {'use_sim_time': use_sim_time}],
            ),
        ],
        condition=IfCondition(
            # mode == 'slam'  →  evaluate at runtime via shell substitution
            # LaunchConfiguration comparison must use EqualsSubstitution
            _mode_is('slam', mode),
        ),
    )

    # ── Nav2 bringup (both modes) ────────────────────────────────────────
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_pkg, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file':  nav2_params,
            'use_composition': 'false',
        }.items(),
    )

    # ── map_server / AMCL (only in nav mode) ─────────────────────────────
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_pkg, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file':  nav2_params,
            'map':          map_path,
        }.items(),
        condition=IfCondition(_mode_is('nav', mode)),
    )

    return LaunchDescription([
        mode_arg,
        map_arg,
        sim_arg,
        ekf_node,
        slam_group,
        localization_launch,
        nav2_launch,
    ])


def _mode_is(value: str, config: LaunchConfiguration):
    """Return an EqualsSubstitution-compatible condition string."""
    from launch.substitutions import PythonExpression
    return PythonExpression(["'", config, "' == '", value, "'"])
