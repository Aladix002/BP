#!/usr/bin/env python3
"""Manuálna jazda (waverower) + voliteľne kamera, IMU (mpu6050driver), LD19, teleop."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_camera = LaunchConfiguration("use_camera")
    use_imu = LaunchConfiguration("use_imu")
    use_lidar = LaunchConfiguration("use_lidar")
    use_teleop = LaunchConfiguration("use_teleop")

    ldlidar_share = get_package_share_directory("ldlidar_ros2")
    ld19_launch = os.path.join(ldlidar_share, "launch", "ld19.launch.py")

    mpu6050_share = get_package_share_directory("mpu6050driver")
    mpu6050_launch = os.path.join(mpu6050_share, "launch", "mpu6050driver_launch.py")

    lidar_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ld19_launch),
        condition=IfCondition(use_lidar),
    )

    imu_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mpu6050_launch),
        condition=IfCondition(use_imu),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_camera",
                default_value="false",
                description="Spusti waverower_camera (subscribe /camera/image_raw).",
            ),
            DeclareLaunchArgument(
                "use_imu",
                default_value="false",
                description="Include balík mpu6050driver (publikuje /imu).",
            ),
            DeclareLaunchArgument(
                "use_lidar",
                default_value="false",
                description="Include ldlidar_ros2 ld19.launch.py (scan na /scan).",
            ),
            DeclareLaunchArgument(
                "use_teleop",
                default_value="false",
                description="teleop_twist_keyboard → /teleop_cmd_vel (potrebuje balík teleop_twist_keyboard).",
            ),
            DeclareLaunchArgument("i2c_bus", default_value="1"),
            DeclareLaunchArgument("i2c_address", default_value="64"),
            DeclareLaunchArgument(
                "control_mode",
                default_value="manual",
                description="manual | auto",
            ),
            Node(
                package="waverower",
                executable="waverower",
                name="wasd_motor_hat_node",
                output="screen",
                parameters=[
                    {
                        "control_mode": LaunchConfiguration("control_mode"),
                        "i2c_bus": LaunchConfiguration("i2c_bus"),
                        "i2c_address": LaunchConfiguration("i2c_address"),
                    }
                ],
            ),
            Node(
                package="waverower",
                executable="waverower_camera",
                name="waverower_camera",
                output="screen",
                condition=IfCondition(use_camera),
            ),
            Node(
                package="teleop_twist_keyboard",
                executable="teleop_twist_keyboard",
                name="teleop_twist_keyboard",
                output="screen",
                remappings=[("cmd_vel", "/teleop_cmd_vel")],
                condition=IfCondition(use_teleop),
            ),
            imu_include,
            lidar_include,
        ]
    )
