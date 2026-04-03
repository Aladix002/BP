#!/usr/bin/env python3
"""Autonómna navigácia: Nav2 + SLAM Toolbox + odometria z /cmd_vel + LD19 LiDAR + IMU.

Módy:
  Bez mapy  (default) – SLAM mapuje + naviguje súčasne
  So mapou            – slam_toolbox v lokalizačnom móde (uložená mapa .yaml)

Príklady:
  ros2 launch waverower nav.launch.py
  ros2 launch waverower nav.launch.py map_yaml:=/home/aladix/mapa.yaml
  ros2 launch waverower nav.launch.py use_rviz:=true

Cieľ poslať cez ROS2 Action (po spustení):
  ros2 run waverower send_goal.py -- 1.5 0.0 0.0
  alebo v RViz: tlačidlo "2D Nav Goal"
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, GroupAction,
    IncludeLaunchDescription, SetEnvironmentVariable, TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg         = get_package_share_directory("waverower")
    ldlidar_share = get_package_share_directory("ldlidar_ros2")
    urdf_xacro  = "/home/aladix/Desktop/BP/gazebo/waver_sim/description/robot.urdf.xacro"

    nav2_params  = os.path.join(pkg, "params", "nav2.yaml")
    slam_params  = os.path.join(pkg, "params", "slam.yaml")
    ld19_launch  = os.path.join(ldlidar_share, "launch", "ld19.launch.py")

    map_yaml   = LaunchConfiguration("map_yaml")
    use_rviz   = LaunchConfiguration("use_rviz")
    imu_port   = LaunchConfiguration("imu_serial_port")
    imu_baud   = LaunchConfiguration("imu_baud_rate")

    use_map = PythonExpression(['"', map_yaml, '" != ""'])

    # Nav2 lifecycle nodes – spravuje lifecycle_manager
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
            description="Cesta k uloženej mape .yaml. Prázdne = SLAM mapping mode.",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
            description="Spusti RViz2 s Nav2 pluginmi.",
        ),
        DeclareLaunchArgument("imu_serial_port", default_value="/dev/ttyACM0"),
        DeclareLaunchArgument("imu_baud_rate",   default_value="115200"),
        DeclareLaunchArgument("i2c_bus",         default_value="1"),
        DeclareLaunchArgument("i2c_address",     default_value="64"),

        # ── Motor node (auto mód – počúva /cmd_vel z Nav2) ────────────────────
        Node(
            package="waverower",
            executable="waverower_motor",
            name="motor_hat_node",
            output="screen",
            parameters=[{
                "control_mode":    "auto",
                "i2c_bus":         LaunchConfiguration("i2c_bus"),
                "i2c_address":     LaunchConfiguration("i2c_address"),
                "imu_correction":  True,
                "imu_yaw_kp":      0.15,
                "imu_yaw_ki":      0.05,
                "imu_yaw_kd":      0.01,
                "imu_yaw_deadband": 0.02,
                "imu_yaw_integral_limit": 0.3,
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
        # odom→base_link: uzol cmd_vel_odometry (integrácia /cmd_vel), nie EKF-only IMU.
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_link_to_imu",
            arguments=["0", "0", "0.05", "0", "0", "0", "base_link", "imu_link"],
        ),

        # ── Robot model (URDF → robot_state_publisher → RobotModel v RViz) ──────
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": Command(["xacro ", urdf_xacro]),
                "use_sim_time": False,
            }],
        ),

        # ── LiDAR LD19 ────────────────────────────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ld19_launch),
        ),

        # ── Odometria bez enkodérov: integrácia /cmd_vel → /odom + TF odom→base_link
        # (EKF len s gyro nedáva x/y → Nav2 „Failed to make progress“.)
        Node(
            package="waverower",
            executable="cmd_vel_odometry.py",
            name="cmd_vel_odometry",
            output="screen",
            parameters=[{"publish_rate": 50.0, "cmd_vel_timeout_sec": 0.5}],
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
        # SLAM lifecycle len v mapping móde (async_slam). Inak by ros2 lifecycle mieril na localization uzol.
        TimerAction(period=10.0, actions=[
            GroupAction(
                actions=[
                    ExecuteProcess(
                        cmd=["ros2", "lifecycle", "set", "/slam_toolbox", "configure"],
                        output="screen",
                    ),
                ],
                condition=UnlessCondition(use_map),
            ),
        ]),
        TimerAction(period=15.0, actions=[
            GroupAction(
                actions=[
                    ExecuteProcess(
                        cmd=["ros2", "lifecycle", "set", "/slam_toolbox", "activate"],
                        output="screen",
                    ),
                ],
                condition=UnlessCondition(use_map),
            ),
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

        # ── Nav2 (oneskorenie ~20 s: kým SLAM configure+activate prebehne a začne publikovať map TF)
        TimerAction(period=20.0, actions=[
            GroupAction([
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
                        ("cmd_vel", "cmd_vel_nav"),
                        ("cmd_vel_smoothed", "/cmd_vel"),
                    ],
                ),
                Node(
                    package="nav2_lifecycle_manager",
                    executable="lifecycle_manager",
                    name="lifecycle_manager_navigation",
                    output="screen",
                    parameters=[{
                        "autostart": True,
                        "node_names": nav2_lifecycle_nodes,
                    }],
                ),
            ]),
        ]),

        # ── RViz2 s Nav2 pluginmi (voliteľný) ─────────────────────────────────
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            condition=IfCondition(use_rviz),
        ),
    ])
