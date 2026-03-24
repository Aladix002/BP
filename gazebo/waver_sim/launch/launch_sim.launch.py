#!/usr/bin/env python3
"""
waver_sim – Gazebo Harmonic simulation with SLAM + Nav2.

Usage:
    ros2 launch waver_sim launch_sim.launch.py

Timeline:
    t=0s   – Gazebo Harmonic, robot_state_publisher, spawn robot
    t=2s   – ros_gz_bridge
    t=10s  – Nav2 bringup (includes SLAM toolbox)
    t=13s  – RViz2
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    pkg = FindPackageShare('waver_sim')

    # ── Robot description ──────────────────────────────────────────────
    robot_description = ParameterValue(
        Command(['xacro ',
                 PathJoinSubstitution([pkg, 'description', 'robot.urdf.xacro'])]),
        value_type=str,
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
    )

    # ── Gazebo Harmonic ────────────────────────────────────────────────
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py']
            )
        ),
        launch_arguments={
            'gz_args': ['-r -v 3 ',
                        PathJoinSubstitution(
                            [FindPackageShare('waver_gazebo'), 'worlds', 'room.sdf']
                        )],
        }.items(),
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_articubot',
        arguments=[
            '-name', 'waver_sim',
            '-topic', 'robot_description',
            '-z', '0.01',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    # ── Twist mux (keyboard priority > Nav2) ──────────────────────────
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        parameters=[
            PathJoinSubstitution([pkg, 'config', 'twist_mux_params.yaml']),
            {'use_sim_time': use_sim_time},
        ],
        remappings=[('cmd_vel_out', '/cmd_vel')],
        output='screen',
    )

    # ── ROS ↔ Gazebo bridge ────────────────────────────────────────────
    gz_bridge = TimerAction(
        period=2.0,
        actions=[Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_bridge',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                '--ros-args', '-p',
                ['config_file:=',
                 PathJoinSubstitution([pkg, 'config', 'ros_gz_bridge.yaml'])],
            ],
        )],
    )

    # ── Nav2 bringup (SLAM + navigation) ──────────────────────────────
    nav2_bringup = TimerAction(
        period=10.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py']
                )
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'slam': 'True',
                'params_file': PathJoinSubstitution(
                    [pkg, 'config', 'nav2_params.yaml']
                ),
                'slam_params_file': PathJoinSubstitution(
                    [pkg, 'config', 'slam_params.yaml']
                ),
                'autostart': 'true',
            }.items(),
        )],
    )

    # ── RViz ──────────────────────────────────────────────────────────
    rviz = TimerAction(
        period=13.0,
        actions=[Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', PathJoinSubstitution([pkg, 'config', 'main.rviz'])],
            parameters=[{'use_sim_time': use_sim_time}],
        )],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        rsp,
        gz_sim,
        spawn,
        twist_mux,
        gz_bridge,
        nav2_bringup,
        rviz,
    ])
