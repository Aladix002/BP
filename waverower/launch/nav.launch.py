#!/usr/bin/env python3
"""Autonómna navigácia pre fyzického robota: SLAM + Nav2 + EKF + LD19 + IMU + motory.

Módy:
  Bez mapy  (default) – SLAM mapuje + naviguje súčasne (slam_toolbox async mapping)
  So mapou            – slam_toolbox v lokalizačnom móde (uložená mapa .yaml)

Príklady:
  ros2 launch waverower nav.launch.py
  ros2 launch waverower nav.launch.py use_rviz:=true
  ros2 launch waverower nav.launch.py map_yaml:=/home/aladix/mapa.yaml use_rviz:=true

Posielanie cieľa:
  V RViz: tlačidlo "2D Goal Pose" (publikuje /goal_pose → bt_navigator)
  CLI:    ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \
            '{header: {frame_id: map}, pose: {position: {x: 1.5, y: 0.0}, orientation: {w: 1.0}}}'

Uloženie mapy:
  ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
    "{name: {data: '/home/aladix/mapa'}}"
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg          = get_package_share_directory("waverower")
    ldlidar_share = get_package_share_directory("ldlidar_ros2")

    nav2_params  = os.path.join(pkg, "params", "nav2.yaml")
    ekf_params   = os.path.join(pkg, "params", "ekf.yaml")
    slam_params  = os.path.join(pkg, "params", "slam.yaml")
    nav2_rviz    = os.path.join(pkg, "params", "nav2.rviz")
    ld19_launch  = os.path.join(ldlidar_share, "launch", "ld19.launch.py")

    map_yaml   = LaunchConfiguration("map_yaml")
    use_rviz   = LaunchConfiguration("use_rviz")
    imu_port   = LaunchConfiguration("imu_serial_port")
    imu_baud   = LaunchConfiguration("imu_baud_rate")

    # use_map: true keď je zadaná cesta k mape
    use_map = PythonExpression(['"', map_yaml, '" != ""'])

    nav2_lifecycle_nodes = [
        "controller_server",
        "planner_server",
        "behavior_server",
        "smoother_server",
        "bt_navigator",
        "waypoint_follower",
        "velocity_smoother",
    ]

    return LaunchDescription([
        SetEnvironmentVariable(name="ROS_DOMAIN_ID", value="0"),

        # ── Argumenty ──────────────────────────────────────────────────────────
        DeclareLaunchArgument(
            "map_yaml",
            default_value="",
            description="Cesta k uloženej mape .yaml. Prázdne = SLAM mapping mód.",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
            description="Spusti RViz2 s nav2.rviz (odporúča sa spustiť na PC, nie RPi).",
        ),
        DeclareLaunchArgument(
            "imu_serial_port",
            default_value="/dev/ttyACM0",
            description="Sériový port Arduino IMU.",
        ),
        DeclareLaunchArgument("imu_baud_rate",  default_value="115200"),
        DeclareLaunchArgument("i2c_bus",        default_value="1"),
        DeclareLaunchArgument("i2c_address",    default_value="64"),
        DeclareLaunchArgument(
            "teleop_max_linear",
            default_value="0.30",
            description="Max rýchlosť [m/s] – musí sedieť s nav2.yaml vx_max.",
        ),
        DeclareLaunchArgument(
            "teleop_max_angular",
            default_value="1.0",
            description="Max uhlová rýchlosť [rad/s] – musí sedieť s nav2.yaml wz_max.",
        ),

        # ── Motor node – auto mód (počúva /cmd_vel z Nav2) ───────────────────
        Node(
            package="waverower",
            executable="waverower_motor",
            name="motor_hat_node",
            output="screen",
            parameters=[{
                "control_mode":           "auto",
                "i2c_bus":                LaunchConfiguration("i2c_bus"),
                "i2c_address":            LaunchConfiguration("i2c_address"),
                "pwm_min":                400,
                "pwm_max":                4095,
                "teleop_max_linear":      LaunchConfiguration("teleop_max_linear"),
                "teleop_max_angular":     LaunchConfiguration("teleop_max_angular"),
                "wheel_base":             0.20,
                # Nav2 /cmd_vel: kladné linear.x = dopredu; robot má invertovanú logiku
                "cmd_vel_invert_linear":  True,
                "smooth_alpha":           0.15,
                # IMU korekcia priamej jazdy (PID na yaw rate gyra)
                "imu_correction":         True,
                "imu_kp":                 0.30,
                "imu_ki":                 0.05,
                "imu_kd":                 0.01,
                "imu_deadband":           0.02,
                "imu_windup":             0.30,
                "imu_sign":               -1.0,
            }],
        ),

        # ── IMU: Arduino USB → /imu ────────────────────────────────────────────
        Node(
            package="waverower",
            executable="imu_serial_fusion_bridge.py",
            name="imu_serial_fusion_bridge",
            output="screen",
            parameters=[{
                "serial_port": imu_port,
                "baud_rate":   imu_baud,
                "frame_id":    "imu_link",
                "topic":       "/imu",
            }],
        ),

        # ── Statické TF ────────────────────────────────────────────────────────
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_link_to_imu",
            output="screen",
            arguments=["0", "0", "0.05", "0", "0", "0", "base_link", "imu_link"],
        ),

        # ── LiDAR LD19 ─────────────────────────────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ld19_launch),
        ),

        # ── EKF: IMU yaw rate → /odom + odom→base_link TF ────────────────────
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[ekf_params],
            remappings=[("odometry/filtered", "/odom")],
        ),

        # ── SLAM Toolbox: mapping mód (bez mapy) ──────────────────────────────
        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            condition=UnlessCondition(use_map),
            parameters=[slam_params],
        ),
        # Lifecycle: configure po 3 s, activate po 8 s
        TimerAction(period=3.0, actions=[
            ExecuteProcess(
                cmd=["ros2", "lifecycle", "set", "/slam_toolbox", "configure"],
                output="screen",
            )
        ]),
        TimerAction(period=8.0, actions=[
            ExecuteProcess(
                cmd=["ros2", "lifecycle", "set", "/slam_toolbox", "activate"],
                output="screen",
            )
        ]),

        # ── SLAM Toolbox: lokalizačný mód (so mapou) ──────────────────────────
        Node(
            package="slam_toolbox",
            executable="localization_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            condition=IfCondition(use_map),
            parameters=[slam_params, {"map_file_name": map_yaml, "mode": "localization"}],
        ),

        # ── Nav2 nodes ─────────────────────────────────────────────────────────
        Node(
            package="nav2_controller",
            executable="controller_server",
            output="screen",
            parameters=[nav2_params],
            remappings=[("cmd_vel", "/cmd_vel")],
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[nav2_params],
        ),
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            parameters=[nav2_params],
        ),
        Node(
            package="nav2_smoother",
            executable="smoother_server",
            name="smoother_server",
            output="screen",
            parameters=[nav2_params],
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=[nav2_params],
        ),
        Node(
            package="nav2_waypoint_follower",
            executable="waypoint_follower",
            name="waypoint_follower",
            output="screen",
            parameters=[nav2_params],
        ),
        Node(
            package="nav2_velocity_smoother",
            executable="velocity_smoother",
            name="velocity_smoother",
            output="screen",
            parameters=[nav2_params],
            remappings=[
                ("cmd_vel",         "cmd_vel_nav"),
                ("cmd_vel_smoothed", "/cmd_vel"),
            ],
        ),

        # ── Nav2 Lifecycle Manager ─────────────────────────────────────────────
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[{
                "autostart":   True,
                "node_names":  nav2_lifecycle_nodes,
            }],
        ),

        # ── RViz2 s Nav2 displejmi (voliteľný, odporúča sa na PC) ─────────────
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            condition=IfCondition(use_rviz),
            arguments=["-d", nav2_rviz],
        ),
    ])
