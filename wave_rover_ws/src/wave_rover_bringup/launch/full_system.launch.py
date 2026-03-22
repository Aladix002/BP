"""
full_system.launch.py  –  Complete Wave Rover system in a single command

Launches everything:
  1. Sensors:    imu_node, camera_node
  2. Control:    watchdog, imu_stabilizer, mode_manager, motor_controller
  3. Navigation: robot_localization EKF, SLAM / Nav2
  4. Teleop:     teleop_twist_keyboard (optional, in a new terminal window hint)

Usage:
  # Full stack in SLAM (mapping) mode
  ros2 launch wave_rover_bringup full_system.launch.py

  # Full stack in navigation mode with pre-built map
  ros2 launch wave_rover_bringup full_system.launch.py nav_mode:=nav map:=/path/to/map.yaml

  # With mock motors (no hardware)
  ros2 launch wave_rover_bringup full_system.launch.py use_mock:=true

Arguments:
  nav_mode       slam | nav         default: slam
  map            path to .yaml      required if nav_mode:=nav
  use_camera     true | false       default: true
  use_imu_stab   true | false       default: true
  use_rviz       true | false       default: true
  use_mock        true | false       default: false
  initial_mode   manual | auto      default: manual
  esp32_ip       IP address         default: 192.168.0.224
  imu_backend    http | i2c         default: http
  use_sim_time   true | false       default: false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                             LogInfo, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_pkg = get_package_share_directory('wave_rover_bringup')

    # ── arguments ─────────────────────────────────────────────────────────
    args = [
        DeclareLaunchArgument('nav_mode',      default_value='slam'),
        DeclareLaunchArgument('map',           default_value=''),
        DeclareLaunchArgument('use_camera',    default_value='true'),
        DeclareLaunchArgument('use_imu_stab',  default_value='true'),
        DeclareLaunchArgument('use_rviz',      default_value='true'),
        DeclareLaunchArgument('use_mock',      default_value='false'),
        DeclareLaunchArgument('initial_mode',  default_value='manual'),
        DeclareLaunchArgument('esp32_ip',      default_value='192.168.0.224'),
        DeclareLaunchArgument('imu_backend',   default_value='http'),
        DeclareLaunchArgument('use_sim_time',  default_value='false'),
    ]

    nav_mode     = LaunchConfiguration('nav_mode')
    map_path     = LaunchConfiguration('map')
    use_camera   = LaunchConfiguration('use_camera')
    use_imu_stab = LaunchConfiguration('use_imu_stab')
    use_rviz     = LaunchConfiguration('use_rviz')
    use_mock     = LaunchConfiguration('use_mock')
    initial_mode = LaunchConfiguration('initial_mode')
    esp32_ip     = LaunchConfiguration('esp32_ip')
    imu_backend  = LaunchConfiguration('imu_backend')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # ── 1. Base bringup (sensors + control) ──────────────────────────────
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'bringup.launch.py')
        ),
        launch_arguments={
            'use_camera':         use_camera,
            'use_imu_stabilizer': use_imu_stab,
            'use_mock':           use_mock,
            'initial_mode':       initial_mode,
            'esp32_ip':           esp32_ip,
            'imu_backend':        imu_backend,
        }.items(),
    )

    # ── 2. Navigation stack (delayed by 3s to let sensors start first) ───
    nav_launch = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup_pkg, 'launch', 'navigation.launch.py')
                ),
                launch_arguments={
                    'mode':         nav_mode,
                    'map':          map_path,
                    'use_rviz':     use_rviz,
                    'use_sim_time': use_sim_time,
                }.items(),
            )
        ],
    )

    return LaunchDescription([
        *args,
        LogInfo(msg='=== Wave Rover Full System Bringup ==='),
        LogInfo(msg=['  nav_mode=',  nav_mode,
                     '  initial_mode=', initial_mode,
                     '  use_mock=',  use_mock]),
        LogInfo(msg='Teleop hint: ros2 run teleop_twist_keyboard '
                    'teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel_teleop'),
        LogInfo(msg='Mode switch: ros2 topic pub /mode std_msgs/String data:\\ auto\\ '),
        bringup,
        nav_launch,
    ])
