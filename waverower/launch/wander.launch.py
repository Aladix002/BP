#!/usr/bin/env python3
"""Autonómne bludenie: LiDAR sektory + IMU otočenie + IMU PID priama jazda.

Robot jazdí autonómne, vyhýba sa prekážkam – bez mapy, bez Nav2.
  • Predok voľný  → jazdi rovno  (motor_hat IMU PID zarovnáva)
  • Prekážka      → porovnaj L/R sektory → otočenie ~90° (IMU meria uhol)

Dynamická rekonf. (ROS2 demo):
  ros2 param set /lidar_wander_node enabled false   ← zastav bludenie
  ros2 param set /lidar_wander_node obstacle_dist_m 0.6
  ros2 param set /lidar_wander_node turn_angle_deg 120.0
  ros2 param set /motor_hat_node imu_correction true
  ros2 param set /motor_hat_node imu_yaw_kp 0.20

Spustenie:
  ros2 launch waverower wander.launch.py
  ros2 launch waverower wander.launch.py obstacle_dist_m:=0.7
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg          = get_package_share_directory("waverower")
    ldlidar_share = get_package_share_directory("ldlidar_ros2")
    ld19_launch  = os.path.join(ldlidar_share, "launch", "ld19.launch.py")

    return LaunchDescription([
        SetEnvironmentVariable(name="ROS_DOMAIN_ID", value="0"),

        # ── Argumenty ──────────────────────────────────────────────────────────
        DeclareLaunchArgument("i2c_bus",           default_value="1"),
        DeclareLaunchArgument("i2c_address",       default_value="64"),
        DeclareLaunchArgument("imu_serial_port",   default_value="/dev/ttyACM0"),
        DeclareLaunchArgument("imu_baud_rate",     default_value="115200"),
        DeclareLaunchArgument(
            "threshold_m", default_value="0.30",
            description="Prah vzdialenosti pre všetky kvadranty [m].",
        ),
        DeclareLaunchArgument(
            "forward_speed", default_value="0.10",
            description="Rýchlosť priamej jazdy [m/s].",
        ),
        DeclareLaunchArgument(
            "turn_speed", default_value="1.80",
            description="Uhlová rýchlosť otáčania [rad/s].",
        ),
        DeclareLaunchArgument(
            "lidar_rotation_deg", default_value="90.0",
            description="Fyzická rotácia LiDARu voči robotu [°]. 90 = keď je predok detegovaný ako ľavý sektor.",
        ),

        # ── Motor node – AUTO mód (počúva /cmd_vel z wander uzla) ────────────
        # IMU PID korekcia aktívna pri priamej jazde
        Node(
            package="waverower",
            executable="waverower",
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
                "serial_port": LaunchConfiguration("imu_serial_port"),
                "baud_rate":   LaunchConfiguration("imu_baud_rate"),
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

        # ── LiDAR LD19 (publikuje /scan + TF base_link→base_laser) ───────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ld19_launch),
        ),

        # ── Wander node ────────────────────────────────────────────────────────
        Node(
            package="waverower",
            executable="lidar_wander.py",
            name="lidar_wander_node",
            output="screen",
            parameters=[{
                "enabled":            True,
                "threshold_m":        LaunchConfiguration("threshold_m"),
                "forward_speed":      LaunchConfiguration("forward_speed"),
                "turn_speed":         LaunchConfiguration("turn_speed"),
                "lidar_rotation_deg": LaunchConfiguration("lidar_rotation_deg"),
                "cmd_topic":          "/cmd_vel",
            }],
        ),
    ])
