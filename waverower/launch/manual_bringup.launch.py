#!/usr/bin/env python3
"""Manuálna jazda (waverower) + kamera (camera_ros) + voliteľne IMU (mpu6050driver), LD19, teleop."""

import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    use_camera = LaunchConfiguration("use_camera")
    use_imu    = LaunchConfiguration("use_imu")
    use_lidar  = LaunchConfiguration("use_lidar")
    use_teleop = LaunchConfiguration("use_teleop")
    use_flow   = LaunchConfiguration("use_flow")
    flow_algo  = LaunchConfiguration("flow_algo")

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

    try:
        get_package_share_directory("camera_ros")
        have_camera_ros = True
    except PackageNotFoundError:
        have_camera_ros = False

    camera_stack = []
    if have_camera_ros:
        # camera_ros publikuje CompressedImage na
        # /camera/camera_node/image_raw/compressed (namespace + node name).
        camera_stack = [
            Node(
                package="camera_ros",
                executable="camera_node",
                name="camera_node",
                namespace="camera",
                output="screen",
                condition=IfCondition(use_camera),
                remappings=[
                    ("image_raw", "/camera/image_raw"),
                    ("camera_info", "/camera/camera_info"),
                ],
            )
        ]
    else:
        camera_stack = [
            LogInfo(
                condition=IfCondition(use_camera),
                msg=(
                    "use_camera:=true vyžaduje nainštalovaný balík camera_ros "
                    "(napr. sudo apt install ros-jazzy-camera-ros)."
                ),
            )
        ]

    # Keď use_flow:=true, motor node číta z /teleop_cmd_vel_corrected (výstup optical_flow_node).
    # Inak číta priamo z /teleop_cmd_vel.
    motor_twist_topic = PythonExpression([
        '"/teleop_cmd_vel_corrected" if "', use_flow, '" == "true" else "/teleop_cmd_vel"'
    ])

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_camera",
                default_value="false",
                description="Spusti camera_ros; web UI berie /camera/camera_node/image_raw/compressed.",
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
            DeclareLaunchArgument(
                "use_flow",
                default_value="false",
                description="Optical flow korekcia jazdy podľa kamery.",
            ),
            DeclareLaunchArgument(
                "flow_algo",
                default_value="lk",
                description="Algoritmus optical flow: lk (Lucas-Kanade sparse) | farneback (dense).",
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
                parameters=[{
                    "control_mode":       LaunchConfiguration("control_mode"),
                    "i2c_bus":            LaunchConfiguration("i2c_bus"),
                    "i2c_address":        LaunchConfiguration("i2c_address"),
                    "manual_twist_topic": motor_twist_topic,
                }],
            ),
            Node(
                package="waverower",
                executable="optical_flow",
                name="optical_flow_node",
                output="screen",
                condition=IfCondition(
                    PythonExpression(['"', use_flow, '" == "true" and "', flow_algo, '" != "farneback"'])
                ),
                parameters=[{
                    "correction_gain":   1.5,
                    "max_correction":    0.3,
                    "forward_threshold": 0.05,
                    "steer_deadzone":    0.12,
                    "min_features":      15,
                }],
            ),
            Node(
                package="waverower",
                executable="optical_flow_dense",
                name="optical_flow_node",
                output="screen",
                condition=IfCondition(
                    PythonExpression(['"', use_flow, '" == "true" and "', flow_algo, '" == "farneback"'])
                ),
                parameters=[{
                    "correction_gain":   1.5,
                    "max_correction":    0.3,
                    "forward_threshold": 0.05,
                    "steer_deadzone":    0.12,
                }],
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
            *camera_stack,
        ]
    )
