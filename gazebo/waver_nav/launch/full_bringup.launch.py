#!/usr/bin/env python3
"""
WaveRover full bringup — Gazebo + SLAM + Nav2 + RViz with manual/auto switching.

MODE SWITCHING
──────────────
  With twist_mux installed (recommended):
    Manual    (keyboard): open a NEW terminal and run
                  ros2 run teleop_twist_keyboard teleop_twist_keyboard \\
                      --ros-args -r /cmd_vel:=/cmd_vel_key
                While a key is held the robot responds immediately.
                Releasing all keys for >0.5 s hands control back to Nav2.

  Without twist_mux:
    Nav2 publishes directly to /cmd_vel. Keyboard teleop would fight Nav2 on the same
    topic — install ros-jazzy-twist-mux (see waver-humble/install_deps.sh) for blending.

  Autonomous (Nav2):    use the "Nav2 Goal" button (arrow icon) in RViz,
            then click anywhere on the map.

Usage:
    ros2 launch waver_nav full_bringup.launch.py
"""

import os

from ament_index_python.packages import PackageNotFoundError, get_package_prefix
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from nav2_common.launch import RewrittenYaml


def _instructions_with_mux():
    return """
╔══════════════════════════════════════════════════════════════════╗
║  WaveRover ready.  Two control modes:                            ║
║                                                                  ║
║  MANUAL  →  new terminal:                                        ║
║    ros2 run teleop_twist_keyboard teleop_twist_keyboard \\        ║
║        --ros-args -r /cmd_vel:=/cmd_vel_key                      ║
║    Hold a key to drive.  Release >0.5 s → Nav2 takes over.       ║
║                                                                  ║
║  AUTONOMOUS  →  in RViz click the Nav2 Goal arrow, then         ║
║    click on the map.  Nav2 plans & drives there.                 ║
╚══════════════════════════════════════════════════════════════════╝
"""


def _instructions_no_mux():
    return """
╔══════════════════════════════════════════════════════════════════╗
║  WaveRover ready (Nav2 → /cmd_vel).                              ║
║  twist_mux is not installed; keyboard + Nav2 mux is disabled.   ║
║  Install: sudo apt install ros-jazzy-twist-mux                     ║
║  AUTONOMOUS → Nav2 Goal in RViz, click on the map.               ║
╚══════════════════════════════════════════════════════════════════╝
"""


def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time')
    waver_nav_share = get_package_share_directory('waver_nav')
    twist_mux_yaml = os.path.join(waver_nav_share, 'param', 'twist_mux_params.yaml')

    nav2_yaml = context.launch_configurations.get('params_file')
    slam_yaml = context.launch_configurations.get('slam_params_file')
    if not nav2_yaml or not slam_yaml:
        raise RuntimeError('params_file and slam_params_file must be set')

    try:
        get_package_prefix('twist_mux')
        have_mux = True
    except PackageNotFoundError:
        have_mux = False

    if have_mux:
        nav2_params_file = nav2_yaml
        instructions = _instructions_with_mux()
    else:
        nav2_params_file = RewrittenYaml(
            source_file=nav2_yaml,
            param_rewrites={
                'collision_monitor.ros__parameters.cmd_vel_out_topic': 'cmd_vel',
            },
            convert_types=True,
        )
        instructions = _instructions_no_mux()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('waver_gazebo'), 'launch', 'gazebo.launch.py']
            )
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        parameters=[twist_mux_yaml, {'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel_out', '/cmd_vel')],
        output='screen',
    )

    nav2_slam = TimerAction(
        period=10.0,   # wait for Gazebo physics to fully settle before SLAM initialises
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
                    'slam': 'True',
                    'params_file': nav2_params_file,
                    'slam_params_file': slam_yaml,
                    'autostart': 'true',
                }.items(),
            )
        ],
    )

    # Shortly after SLAM starts (t=10.5 s), flood /initialpose at 2 Hz for 10 s so
    # SLAM anchors to (0, 0, 0°) before it builds graph nodes with a spurious heading.
    # The odom accumulates a wrong yaw from wheel spin during the spawn physics-settle;
    # repeated delivery ensures SLAM receives the correction within the first few scans.
    initial_pose_reset = TimerAction(
        period=10.5,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'topic', 'pub',
                    '--rate', '2',
                    '--times', '20',
                    '/initialpose',
                    'geometry_msgs/msg/PoseWithCovarianceStamped',
                    (
                        '{header: {frame_id: map},'
                        ' pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0},'
                        '               orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}},'
                        '        covariance: [0.25,0,0,0,0,0,'
                        '                     0,0.25,0,0,0,0,'
                        '                     0,0,0,0,0,0,'
                        '                     0,0,0,0,0,0,'
                        '                     0,0,0,0,0,0,'
                        '                     0,0,0,0,0,0.068]}}'
                    ),
                ],
                output='screen',
            )
        ],
    )

    rviz = TimerAction(
        period=13.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=[
                    '-d',
                    PathJoinSubstitution(
                        [FindPackageShare('waver_nav'), 'rviz', 'waver_full.rviz']
                    ),
                ],
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen',
            )
        ],
    )

    actions = [
        LogInfo(msg=instructions),
        gazebo,
    ]
    if have_mux:
        actions.append(twist_mux_node)
    else:
        actions.append(
            LogInfo(
                msg='[waver_nav] Package twist_mux not found — '
                'using direct /cmd_vel from Nav2. '
                'For keyboard + Nav2 blending: sudo apt install ros-jazzy-twist-mux'
            )
        )
    actions.extend([nav2_slam, initial_pose_reset, rviz])
    return actions


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('waver_nav'), 'param', 'nav2_params.yaml']
            ),
        ),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('waver_nav'), 'param', 'slam_params.yaml']
            ),
        ),
        OpaqueFunction(function=launch_setup),
    ])
