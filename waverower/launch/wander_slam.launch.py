#!/usr/bin/env python3
"""Autonómne bludenie + online SLAM (jeden launch).

Spustí LiDAR wander, motor (auto), IMU, TF pre SLAM a slam_toolbox (lifecycle configure/activate).
Statický TF odom→base_link (identita) – rovnako ako slam.launch bez EKF.

Na PC (rovnaký ROS_DOMAIN_ID): rviz2 -d .../slam.rviz alebo tu use_rviz:=true.

Príklady:
  ros2 launch waverower wander_slam.launch.py
  ros2 launch waverower wander_slam.launch.py use_rviz:=true
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
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("waverower")
    ldlidar_share = get_package_share_directory("ldlidar_ros2")
    ld19_launch = os.path.join(ldlidar_share, "launch", "ld19.launch.py")
    slam_params = os.path.join(pkg, "params", "slam.yaml")

    use_rviz = LaunchConfiguration("use_rviz")

    return LaunchDescription([
        SetEnvironmentVariable(name="ROS_DOMAIN_ID", value="0"),

        DeclareLaunchArgument("i2c_bus", default_value="1"),
        DeclareLaunchArgument("i2c_address", default_value="64"),
        DeclareLaunchArgument("imu_serial_port", default_value="/dev/ttyACM0"),
        DeclareLaunchArgument("imu_baud_rate", default_value="115200"),
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
            description="Fyzická rotácia LiDARu voči robotu [°].",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
            description="Spusti RViz2 lokálne (na PC často len rviz2 + rovnaký ROS_DOMAIN_ID).",
        ),

        Node(
            package="waverower",
            executable="waverower",
            name="motor_hat_node",
            output="screen",
            parameters=[{
                "control_mode": "auto",
                "i2c_bus": LaunchConfiguration("i2c_bus"),
                "i2c_address": LaunchConfiguration("i2c_address"),
                "imu_correction": True,
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
            name="odom_to_base_link",
            arguments=["0", "0", "0", "0", "0", "0", "odom", "base_link"],
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

        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[slam_params],
        ),
        TimerAction(period=2.0, actions=[
            ExecuteProcess(
                cmd=["ros2", "lifecycle", "set", "/slam_toolbox", "configure"],
                output="screen",
            )
        ]),
        TimerAction(period=4.0, actions=[
            ExecuteProcess(
                cmd=["ros2", "lifecycle", "set", "/slam_toolbox", "activate"],
                output="screen",
            )
        ]),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            condition=IfCondition(use_rviz),
            arguments=["-d", os.path.join(pkg, "params", "slam.rviz")],
        ),
    ])
