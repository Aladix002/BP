#!/usr/bin/env python3
"""SLAM entrypoint: manual_bringup s LiDAR + IMU + SLAM.

Plny stack: ros2 launch waverower manual_bringup.launch.py

Priklady:
  ros2 launch waverower slam.launch.py
  ros2 launch waverower slam.launch.py use_rviz:=true

Ulozenie mapy:
  ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '/home/user/mapa'}}"
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg = get_package_share_directory("waverower")
    manual = os.path.join(pkg, "launch", "manual_bringup.launch.py")
    return LaunchDescription(
        [
            SetEnvironmentVariable(name="ROS_DOMAIN_ID", value="0"),
            DeclareLaunchArgument(
                "correction_mode",
                default_value="imu",
                description="imu | none",
            ),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument(
                "imu_serial_port",
                default_value="/dev/ttyACM0",
                description="Arduino IMU (ina default cesta ako manual_bringup)",
            ),
            DeclareLaunchArgument("imu_baud_rate", default_value="115200"),
            DeclareLaunchArgument("use_camera", default_value="false"),
            DeclareLaunchArgument("i2c_bus", default_value="1"),
            DeclareLaunchArgument("i2c_address", default_value="64"),
            DeclareLaunchArgument("use_ekf", default_value="false"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(manual),
                launch_arguments=[
                    ("correction_mode", LaunchConfiguration("correction_mode")),
                    ("use_rviz", LaunchConfiguration("use_rviz")),
                    ("imu_serial_port", LaunchConfiguration("imu_serial_port")),
                    ("imu_baud_rate", LaunchConfiguration("imu_baud_rate")),
                    ("use_camera", LaunchConfiguration("use_camera")),
                    ("i2c_bus", LaunchConfiguration("i2c_bus")),
                    ("i2c_address", LaunchConfiguration("i2c_address")),
                    ("use_ekf", LaunchConfiguration("use_ekf")),
                    ("use_slam", "true"),
                    ("use_lidar", "true"),
                    ("use_imu", "true"),
                    ("control_mode", "manual"),
                    ("use_teleop", "false"),
                    ("use_ball_follow", "false"),
                    ("use_imu_kalman", "false"),
                ],
            ),
        ]
    )
