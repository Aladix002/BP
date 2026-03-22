#!/usr/bin/env python3
"""
Autonomous navigation bringup for WaveRover simulation.

Starts: Gazebo + WaveRover + AMCL localisation + Nav2 + RViz.
Requires a pre-built map. Pass the map YAML path via:

    ros2 launch waver_nav nav_bringup.launch.py \
        map:=/path/to/your_map.yaml

Then in RViz:
  1. Use "2D Pose Estimate" to initialise AMCL (set robot start pose).
  2. Use "Nav2 Goal" / "Navigation2 Goal Pose" to send a navigation goal.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_yaml = LaunchConfiguration(
        'map',
        default=PathJoinSubstitution(
            [FindPackageShare('waver_nav'), 'maps', 'room.yaml']
        ),
    )
    nav2_params = LaunchConfiguration(
        'params_file',
        default=PathJoinSubstitution(
            [FindPackageShare('waver_nav'), 'param', 'nav2_params.yaml']
        ),
    )
    launch_rviz = LaunchConfiguration('rviz', default='true')

    # Gazebo + robot spawn + bridge
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('waver_gazebo'), 'launch', 'gazebo.launch.py']
            )
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # Full Nav2 stack (map_server + amcl + planner + controller + bt_navigator …)
    nav2 = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare('nav2_bringup'), 'launch',
                         'bringup_launch.py']
                    )
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'map': map_yaml,
                    'params_file': nav2_params,
                    'autostart': 'true',
                }.items(),
            )
        ],
    )

    # RViz with Nav2 panels
    rviz = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                condition=IfCondition(launch_rviz),
                arguments=[
                    '-d',
                    PathJoinSubstitution(
                        [FindPackageShare('waver_nav'), 'rviz', 'waver_nav.rviz']
                    ),
                ],
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen',
            )
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use Gazebo simulation clock'),
        DeclareLaunchArgument(
            'map',
            default_value=PathJoinSubstitution(
                [FindPackageShare('waver_nav'), 'maps', 'room.yaml']
            ),
            description='Full path to the map YAML file',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('waver_nav'), 'param', 'nav2_params.yaml']
            ),
            description='Full path to nav2 params YAML file',
        ),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Launch RViz2'),
        gazebo,
        nav2,
        rviz,
    ])
