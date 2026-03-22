#!/usr/bin/env python3
"""
SLAM mapping bringup for WaveRover simulation.

Starts: Gazebo + WaveRover + SLAM Toolbox (async online) + RViz.
Use teleop_twist_keyboard in a second terminal to drive while building the map:

    ros2 run teleop_twist_keyboard teleop_twist_keyboard

Save the map when done:
    ros2 run nav2_map_server map_saver_cli -f ~/map_name
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    slam_params = LaunchConfiguration(
        'slam_params_file',
        default=PathJoinSubstitution(
            [FindPackageShare('waver_nav'), 'param', 'slam_params.yaml']
        ),
    )

    # Gazebo + robot spawn + bridge
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('waver_gazebo'), 'launch', 'gazebo.launch.py']
            )
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # SLAM Toolbox – wait 3 s for Gazebo/robot to be ready
    slam = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare('slam_toolbox'), 'launch',
                         'online_async_launch.py']
                    )
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'slam_params_file': slam_params,
                }.items(),
            )
        ],
    )

    # RViz with SLAM config
    rviz = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=[
                    '-d',
                    PathJoinSubstitution(
                        [FindPackageShare('waver_nav'), 'rviz', 'waver_slam.rviz']
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
            'slam_params_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('waver_nav'), 'param', 'slam_params.yaml']
            ),
            description='Path to SLAM Toolbox parameter file',
        ),
        gazebo,
        slam,
        rviz,
    ])
