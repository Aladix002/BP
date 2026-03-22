#!/usr/bin/env python3
"""Launch Gazebo Harmonic with the WaveRover spawned inside."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file = LaunchConfiguration(
        'world_file',
        default=PathJoinSubstitution(
            [FindPackageShare('waver_gazebo'), 'worlds', 'room.sdf']
        ),
    )

    # Robot description (robot_state_publisher + joint_state_publisher)
    description_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('waver_description'), 'launch', 'description.launch.xml']
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'sim_control': 'gazebo',
            'camera_type': 'raspi',
            'publish_joint_states': 'false',
            'rsp_ignore_joint_timestamp': 'true',
        }.items(),
    )

    # Gazebo Harmonic
    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py']
            )
        ),
        launch_arguments={
            'gz_args': ['-r -v 3 ', world_file],
        }.items(),
    )

    # Spawn robot from /robot_description topic
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_waver',
        arguments=[
            '-name', 'waver',
            '-topic', 'robot_description',
            '-z', '0.01',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    # Bridge after Gazebo has stepped a few times so /clock is monotonic before /joint_states
    # floods robot_state_publisher (avoids "Moved backwards in time" spam at startup).
    gz_bridge = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='gz_bridge',
                parameters=[{'use_sim_time': use_sim_time}],
                arguments=[
                    '--ros-args',
                    '-p', ['config_file:=',
                           PathJoinSubstitution(
                               [FindPackageShare('waver_gazebo'), 'config',
                                'ros_gz_bridge.yaml']
                           )],
                ],
                output='screen',
            ),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'world_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('waver_gazebo'), 'worlds', 'room.sdf']
            ),
        ),
        description_launch,
        gz_sim_launch,
        spawn_robot,
        gz_bridge,
    ])
