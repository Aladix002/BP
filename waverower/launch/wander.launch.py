#!/usr/bin/env python3
"""Autonomne bludenie: LiDAR + IMU + korekcia priamky. Bez mapy, bez Nav2.

correction_mode: imu | none

Dynamicka rekonf.:
  ros2 param set /lidar_wander_node enabled false

Spustenie: ros2 launch waverower wander.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    ldlidar_share = get_package_share_directory("ldlidar_ros2")
    ld19_launch = os.path.join(ldlidar_share, "launch", "ld19.launch.py")

    correction_mode = LaunchConfiguration("correction_mode")
    use_imu_correction = PythonExpression(['"', correction_mode, '" == "imu"'])

    return LaunchDescription([
        SetEnvironmentVariable(name="ROS_DOMAIN_ID", value="0"),

        DeclareLaunchArgument("i2c_bus", default_value="1"),
        DeclareLaunchArgument("i2c_address", default_value="64"),
        DeclareLaunchArgument("imu_serial_port", default_value="/dev/ttyACM0"),
        DeclareLaunchArgument("imu_baud_rate", default_value="115200"),
        DeclareLaunchArgument(
            "threshold_m", default_value="0.30",
            description="prah vzdialenosti vsetkych kvadrantov [m]",
        ),
        DeclareLaunchArgument(
            "forward_speed", default_value="0.10",
            description="rychlost priamky [m/s]",
        ),
        DeclareLaunchArgument(
            "turn_speed", default_value="1.80",
            description="uhlova rychlost [rad/s]",
        ),
        DeclareLaunchArgument(
            "lidar_rotation_deg", default_value="90.0",
            description="fyzicka rotacia LiDAR voci robotovi [deg]; 90 = predok ako lavy sektor",
        ),
        DeclareLaunchArgument(
            "correction_mode",
            default_value="imu",
            description="imu | none",
        ),
        Node(
            package="waverower",
            executable="waverower_motor",
            name="motor_hat_node",
            output="screen",
            parameters=[{
                "control_mode": "auto",
                "i2c_bus": LaunchConfiguration("i2c_bus"),
                "i2c_address": LaunchConfiguration("i2c_address"),
                "imu_correction": use_imu_correction,
                "imu_yaw_kp": 0.15,
                "imu_yaw_ki": 0.05,
                "imu_yaw_kd": 0.01,
                "imu_yaw_deadband": 0.02,
                "imu_yaw_integral_limit": 0.3,
            }],
        ),

        Node(
            package="waverower",
            executable="imu_serial_fusion_bridge.py",
            name="imu_serial_fusion_bridge",
            output="screen",
            parameters=[{
                "serial_port": LaunchConfiguration("imu_serial_port"),
                "baud_rate": LaunchConfiguration("imu_baud_rate"),
                "frame_id": "imu_link",
                "topic": "/imu",
            }],
        ),

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_link_to_imu",
            arguments=["0", "0", "0.05", "0", "0", "0", "base_link", "imu_link"],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ld19_launch),
        ),


        Node(
            package="waverower",
            executable="lidar_wander.py",
            name="lidar_wander_node",
            output="screen",
            parameters=[{
                "enabled": True,
                "threshold_m": LaunchConfiguration("threshold_m"),
                "forward_speed": LaunchConfiguration("forward_speed"),
                "turn_speed": LaunchConfiguration("turn_speed"),
                "lidar_rotation_deg": LaunchConfiguration("lidar_rotation_deg"),
                "cmd_topic": "/cmd_vel",
            }],
        ),
    ])
