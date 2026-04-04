#!/usr/bin/env python3
"""Autonómna navigácia: Nav2 + SLAM Toolbox + odometria z /cmd_vel+IMU + LD19 LiDAR.

RViz beží na PC (nie tu). Na PC spusti:
  source /opt/ros/jazzy/setup.bash
  export ROS_DOMAIN_ID=0
  rviz2 -d /home/aladix/Desktop/BP/waverower/params/slam.rviz

Príklady:
  ros2 launch waverower nav.launch.py
  ros2 launch waverower nav_launch.py map_yaml:=/home/aladix/mapa.yaml

Cieľ poslať cez:
  ros2 run waverower send_goal.py -- 1.5 0.0 0.0
  alebo v RViz: tlačidlo "2D Goal Pose"
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
    pkg           = get_package_share_directory("waverower")
    ldlidar_share = get_package_share_directory("ldlidar_ros2")
    urdf_xacro    = "/home/aladix/Desktop/BP/gazebo/waver_sim/description/robot.urdf.xacro"

    nav2_params = os.path.join(pkg, "params", "nav2.yaml")
    slam_params = os.path.join(pkg, "params", "slam.yaml")
    ld19_launch = os.path.join(ldlidar_share, "launch", "ld19.launch.py")

    map_yaml = LaunchConfiguration("map_yaml")
    imu_port = LaunchConfiguration("imu_serial_port")
    imu_baud = LaunchConfiguration("imu_baud_rate")

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
            description="Cesta k uloženej mape .yaml. Prázdne = SLAM mapping mode.",
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
                "control_mode":           "auto",
                "i2c_bus":                LaunchConfiguration("i2c_bus"),
                "i2c_address":            LaunchConfiguration("i2c_address"),
                "imu_correction":         True,
                "imu_yaw_kp":             0.15,
                "imu_yaw_ki":             0.05,
                "imu_yaw_kd":             0.01,
                "imu_yaw_deadband":       0.02,
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
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_link_to_imu",
            arguments=["0", "0", "0.05", "0", "0", "0", "base_link", "imu_link"],
        ),

        # ── Robot model (URDF → robot_state_publisher) ─────────────────────────
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

        # ── LiDAR LD19 ─────────────────────────────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ld19_launch),
        ),

        # ── Odometria: /cmd_vel + IMU gyro → /odom + TF odom→base_link ─────────
        Node(
            package="waverower",
            executable="cmd_vel_odometry.py",
            name="cmd_vel_odometry",
            output="screen",
            parameters=[{"publish_rate": 50.0, "cmd_vel_timeout_sec": 0.5}],
        ),

        # ── SLAM Toolbox: mapping mód (bez mapy) ──────────────────────────────
        # async_slam_toolbox_node = lifecycle node, vyžaduje externé configure+activate
        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            condition=UnlessCondition(use_map),
            parameters=[slam_params],
        ),
        # SLAM lifecycle: configure po 15 s (uzol musí byť plne inicializovaný)
        TimerAction(period=15.0, actions=[
            GroupAction(
                condition=UnlessCondition(use_map),
                actions=[ExecuteProcess(
                    cmd=["ros2", "lifecycle", "set", "/slam_toolbox", "configure"],
                    output="screen",
                )],
            ),
        ]),
        # SLAM lifecycle: activate po 20 s
        TimerAction(period=20.0, actions=[
            GroupAction(
                condition=UnlessCondition(use_map),
                actions=[ExecuteProcess(
                    cmd=["ros2", "lifecycle", "set", "/slam_toolbox", "activate"],
                    output="screen",
                )],
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

        # ── Nav2 (oneskorenie 30 s: kým SLAM activate prebehne a začne map TF) ─
        TimerAction(period=30.0, actions=[
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
                        ("cmd_vel",          "cmd_vel_nav"),
                        ("cmd_vel_smoothed", "/cmd_vel"),
                    ],
                ),
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
            ]),
        ]),
    ])
