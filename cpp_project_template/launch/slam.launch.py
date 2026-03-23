#!/usr/bin/env python3
"""
SLAM (slam_toolbox) + LD19 + waverower v manuálnom režime.

Predpoklady: nainštalovaný / zbuildovaný balík ldlidar_ros2 (workspace alebo overlay),
            slam_toolbox z apt (ros-jazzy-slam-toolbox).

Použitie:
  source /opt/ros/jazzy/setup.bash
  source ~/Desktop/BP/install/setup.bash   # + overlay kde je ldlidar_ros2

  ros2 launch waverower slam.launch.py
  ros2 launch waverower slam.launch.py lidar_port:=/dev/ttyUSB1 use_rviz:=true

Mapu ulož (v druhom termináli):
  ros2 run nav2_map_server map_saver_cli -f ~/moja_mapa

Poznámka: bez enkóderov je odom→base_link statická identita; poloha v mape ide cez scan matching.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("waverower")
    slam_params = os.path.join(pkg_dir, "config", "slam_params.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument("lidar_port", default_value="/dev/ttyUSB0"),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument("i2c_bus", default_value="1"),
            DeclareLaunchArgument("i2c_address", default_value="64"),
            DeclareLaunchArgument(
                "launch_motors",
                default_value="true",
                description="false = len LiDAR + SLAM (motory spusti sám: ros2 run waverower waverower)",
            ),
            Node(
                package="ldlidar_ros2",
                executable="ldlidar_ros2_node",
                name="ldlidar_publisher_ld19",
                output="screen",
                parameters=[
                    {
                        "product_name": "LDLiDAR_LD19",
                        "laser_scan_topic_name": "scan",
                        "point_cloud_2d_topic_name": "pointcloud2d",
                        "frame_id": "base_laser",
                        "port_name": LaunchConfiguration("lidar_port"),
                        "serial_baudrate": 230400,
                        "laser_scan_dir": True,
                        "enable_angle_crop_func": False,
                        "range_min": 0.02,
                        "range_max": 12.0,
                    }
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_link_to_base_laser",
                arguments=["0", "0", "0.18", "0", "0", "0", "base_link", "base_laser"],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="odom_to_base_link",
                arguments=["0", "0", "0", "0", "0", "0", "odom", "base_link"],
            ),
            Node(
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                output="screen",
                parameters=[slam_params],
            ),
            Node(
                package="waverower",
                executable="waverower",
                name="wasd_motor_hat_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("launch_motors")),
                parameters=[
                    {
                        "control_mode": "manual",
                        "i2c_bus": LaunchConfiguration("i2c_bus"),
                        "i2c_address": LaunchConfiguration("i2c_address"),
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                condition=IfCondition(LaunchConfiguration("use_rviz")),
            ),
        ]
    )
