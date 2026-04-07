#!/usr/bin/env python3
"""Hlavny launch pre simulaciu.

Pipeline:
1) vygeneruje robot_description z xacro,
2) pusti Gazebo s worldom room.sdf,
3) spawne robota do sveta,
4) zapne twist_mux (teleop ma vyssiu prioritu ako nav2),
5) zapne ros_gz_bridge (topic bridge medzi ROS a Gazebo),
6) po kratkom oneskoreni pusti Nav2 bringup so SLAM,
7) nakoniec pusti RViz s pripravenou konfiguraciou.
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
    # use_sim_time=True: vsetky ROS nody pouzivaju /clock z Gazeba.
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    # Prefix cesty do balika waver_sim (config/, launch/, description/).
    pkg = FindPackageShare('waver_sim')

    # Xacro -> URDF text, ktory posielame do robot_state_publisher.
    robot_description = ParameterValue(
        Command(['xacro ',
                 PathJoinSubstitution([pkg, 'description', 'robot.urdf.xacro'])]),
        value_type=str,
    )

    # robot_state_publisher publikuje TF strom robota podla URDF.
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
    )

    # Spusti samotny Gazebo simulator a nacita world room.sdf z waver_gazebo.
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

    # Vytvori instanciu robota vo svete pod menom "waver_sim".
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

    # Mixuje cmd_vel vstupy:
    # - /cmd_vel_key (teleop) ma prioritu
    # - /cmd_vel_nav (nav2) je fallback.
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

    # Bridge sa spusta po 2s, aby bol Gazebo uz inicializovany.
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

    # Nav2 bringup sa pusta po 10s:
    # - simulator aj bridge uz bezia,
    # - SLAM + nav stack sa rozbieha stabilnejsie.
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

    # RViz po 13s, aby uz boli k dispozicii map/TF topicy.
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
        # Jediny verejny argument launchu.
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        rsp,
        gz_sim,
        spawn,
        twist_mux,
        gz_bridge,
        nav2_bringup,
        rviz,
    ])
