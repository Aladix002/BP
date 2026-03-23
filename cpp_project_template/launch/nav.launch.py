#!/usr/bin/env python3
"""
Plná autonómna navigácia: LiDAR + SLAM + Nav2 + motory (auto režim) + voliteľná kamera.

Spustenie na RPi:
  ros2 launch waverower nav.launch.py
  ros2 launch waverower nav.launch.py lidar_port:=/dev/ttyUSB1 use_rviz:=true

Spustenie RViz na PC (odporúčané):
  rviz2 -d $(ros2 pkg prefix waverower)/share/waverower/rviz/nav.rviz

Výber cieľa: v RViz kliknúť "Nav2 Goal" (2D Nav Goal) → robot automaticky naplánuje trasu.

Prepnutie do manuálneho režimu (dynamická rekonf.):
  ros2 param set /wasd_motor_hat_node control_mode manual
  ros2 param set /wasd_motor_hat_node control_mode auto

Zapnutie IMU korekcie za behu:
  ros2 param set /wasd_motor_hat_node imu_correction true
  ros2 param set /wasd_motor_hat_node imu_yaw_kp 0.15
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("waverower")
    slam_params  = os.path.join(pkg_dir, "config", "slam_params.yaml")
    nav2_params  = os.path.join(pkg_dir, "config", "nav2_params.yaml")

    return LaunchDescription([
        # ── Argumenty ──────────────────────────────────────────────────────────
        DeclareLaunchArgument("lidar_port",          default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("i2c_bus",             default_value="1"),
        DeclareLaunchArgument("i2c_address",         default_value="64"),
        DeclareLaunchArgument("wheel_separation_m",  default_value="0.20"),
        DeclareLaunchArgument("max_wheel_m_s",       default_value="0.26"),
        DeclareLaunchArgument("imu_correction",      default_value="false"),
        DeclareLaunchArgument("use_camera",          default_value="false",
                              description="Spusti camera_node s YOLO (false = len stream)"),
        DeclareLaunchArgument("use_rviz",            default_value="false",
                              description="Spusti RViz na tomto stroji (zvyčajne na PC)"),
        DeclareLaunchArgument("lidar_offset_x",      default_value="0.0",
                              description="Poloha LiDARu voči base_link [m]"),
        DeclareLaunchArgument("lidar_offset_z",      default_value="0.18"),

        # ── LiDAR LD19 ─────────────────────────────────────────────────────────
        Node(
            package="ldlidar_ros2",
            executable="ldlidar_ros2_node",
            name="ldlidar_publisher_ld19",
            output="screen",
            parameters=[{
                "product_name":             "LDLiDAR_LD19",
                "laser_scan_topic_name":    "scan",
                "point_cloud_2d_topic_name": "pointcloud2d",
                "frame_id":                 "base_laser",
                "port_name":                LaunchConfiguration("lidar_port"),
                "serial_baudrate":          230400,
                "laser_scan_dir":           True,
                "enable_angle_crop_func":   False,
                "range_min":                0.02,
                "range_max":                12.0,
            }],
        ),

        # ── TF: base_link → base_laser ─────────────────────────────────────────
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_link_to_base_laser",
            arguments=[
                LaunchConfiguration("lidar_offset_x"), "0",
                LaunchConfiguration("lidar_offset_z"),
                "0", "0", "0",
                "base_link", "base_laser",
            ],
        ),

        # ── SLAM Toolbox (mapovanie + lokalizácia) ─────────────────────────────
        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[
                slam_params,
                {"use_sim_time": False},
            ],
        ),

        # ── Pseudo-odometria (bez enkodérov): statická identita odom→base_link ─
        # slam_toolbox robí map→base_link priamo; Nav2 potrebuje odom frame.
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="odom_to_base_link",
            arguments=["0", "0", "0", "0", "0", "0", "odom", "base_link"],
        ),

        # ── Nav2 stack ─────────────────────────────────────────────────────────
        Node(
            package="nav2_controller",
            executable="controller_server",
            output="screen",
            parameters=[nav2_params],
            remappings=[("cmd_vel", "cmd_vel_nav")],
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            output="screen",
            parameters=[nav2_params],
        ),
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            output="screen",
            parameters=[nav2_params],
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            output="screen",
            parameters=[nav2_params],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[{
                "use_sim_time":    False,
                "autostart":       True,
                "node_names":      [
                    "controller_server",
                    "planner_server",
                    "behavior_server",
                    "bt_navigator",
                ],
                "bond_timeout":    4.0,
            }],
        ),

        # ── Velocity smoother (Nav2 → motory) ─────────────────────────────────
        # Zmäkčuje príkazy z Nav2 pred odoslaním do motorov
        Node(
            package="nav2_velocity_smoother",
            executable="velocity_smoother",
            output="screen",
            parameters=[nav2_params],
            remappings=[
                ("cmd_vel",       "cmd_vel_nav"),
                ("cmd_vel_smoothed", "cmd_vel"),
            ],
        ),

        # ── Motor HAT (auto režim: počúva cmd_vel z Nav2) ─────────────────────
        Node(
            package="waverower",
            executable="waverower",
            name="wasd_motor_hat_node",
            output="screen",
            parameters=[{
                "control_mode":          "auto",
                "i2c_bus":               LaunchConfiguration("i2c_bus"),
                "i2c_address":           LaunchConfiguration("i2c_address"),
                "wheel_separation_m":    LaunchConfiguration("wheel_separation_m"),
                "max_wheel_linear_m_s":  LaunchConfiguration("max_wheel_m_s"),
                "cmd_vel_timeout_ms":    300,
                "imu_correction":        LaunchConfiguration("imu_correction"),
            }],
        ),

        # ── Kamera (voliteľná) ─────────────────────────────────────────────────
        Node(
            package="waverower",
            executable="waverower_camera",
            name="camera_node",
            output="screen",
            condition=IfCondition(LaunchConfiguration("use_camera")),
            parameters=[{
                "publish_compressed": True,
                "use_compressed":     False,
            }],
        ),

        # ── RViz (voliteľný, zvyčajne na PC) ──────────────────────────────────
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        ),
    ])
