#!/usr/bin/env python3
# Simulacia: Gazebo (Ignition) + ros_gz bridge + Nav2 bringup so SLAM + RViz.
# use_sim_time: vsetky nody synchronizuju cas s /clock zo simulatora (nutne pre replay a stabilne TF).

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

    # xacro expanduje include -> jeden velky URDF retazec pre robot_state_publisher aj spawn
    robot_description = ParameterValue(
        Command(['xacro ',
                 PathJoinSubstitution([pkg, 'description', 'robot.urdf.xacro'])]),
        value_type=str,
    )

    # Publikuje TF statickeho robota (jointy z Gazebo bridge doplnia stav)
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
    )

    # gz_sim: nacita world room.sdf z balika waver_gazebo (-r = run, -v = log uroven)
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

    # create: spawne entitu s nazvom waver_sim, geometria z topicu robot_description
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

    # twist_mux: zluci /cmd_vel_nav a /cmd_vel_key do jedneho /cmd_vel (priorita v yaml)
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

    # Bridge: oneskorenie 2 s, kym bezi Gazebo a existuju entity (inak padne na prazdnych topicoch)
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

    # Nav2 + async SLAM: dlhsi delay aby bol map frame a scan stabilne
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

    # RViz neskor: aby sa nacitali displaye az ked existuju /map a TF
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
