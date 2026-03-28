#!/usr/bin/env python3
"""SLAM mapovanie: LD19 LiDAR + MPU-6050 IMU (Arduino USB) + robot_localization EKF.

Prepínanie korekcie jazdy pri priamej jazde:
  correction_mode:=imu          – gyro Z PID (výchozí)
  correction_mode:=optical_flow – Lucas-Kanade optical flow z kamery
  correction_mode:=none         – bez korekcie

Príklady:
  ros2 launch waverower slam.launch.py
  ros2 launch waverower slam.launch.py correction_mode:=optical_flow
  ros2 launch waverower slam.launch.py use_rviz:=true
  # Uloženie mapy po skončení:
  ros2 run nav2_map_server map_saver_cli -f ~/map
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("waverower")
    ldlidar_share = get_package_share_directory("ldlidar_ros2")

    slam_params  = os.path.join(pkg, "params", "slam.yaml")
    ekf_params   = os.path.join(pkg, "params", "ekf.yaml")
    ld19_launch  = os.path.join(ldlidar_share, "launch", "ld19.launch.py")

    correction_mode = LaunchConfiguration("correction_mode")
    use_rviz        = LaunchConfiguration("use_rviz")
    imu_port        = LaunchConfiguration("imu_serial_port")
    imu_baud        = LaunchConfiguration("imu_baud_rate")
    use_camera      = LaunchConfiguration("use_camera")

    # Podmienky pre jednotlivé korekčné režimy
    use_imu_correction = PythonExpression(['"', correction_mode, '" == "imu"'])
    use_flow_correction = PythonExpression(['"', correction_mode, '" == "optical_flow"'])

    # Motor node dostane správny manual_twist_topic podľa correction_mode:
    #   optical_flow → /teleop_cmd_vel_corrected (výstup z optical_flow_node)
    #   imu / none   → /teleop_cmd_vel
    motor_twist_topic = PythonExpression([
        '"/teleop_cmd_vel_corrected" if "', correction_mode, '" == "optical_flow"',
        ' else "/teleop_cmd_vel"'
    ])

    return LaunchDescription([
        SetEnvironmentVariable(name="ROS_DOMAIN_ID", value="0"),
        # ── Argumenty ──────────────────────────────────────────────────────────
        DeclareLaunchArgument(
            "correction_mode",
            default_value="imu",
            description="Korekcia jazdy rovne: imu | optical_flow | none",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
            description="Spusti RViz2 pre vizualizáciu mapy.",
        ),
        DeclareLaunchArgument(
            "imu_serial_port",
            default_value="/dev/ttyACM0",
            description="Sériový port Arduina s MPU-6050.",
        ),
        DeclareLaunchArgument(
            "imu_baud_rate",
            default_value="115200",
        ),
        DeclareLaunchArgument(
            "use_camera",
            default_value="false",
            description="Spusti camera_ros (potrebné pre correction_mode:=optical_flow).",
        ),
        DeclareLaunchArgument("i2c_bus",     default_value="1"),
        DeclareLaunchArgument("i2c_address", default_value="64"),
        DeclareLaunchArgument(
            "use_ekf",
            default_value="false",
            description="Spusti robot_localization EKF (IMU→odom TF). Slam funguje aj bez neho.",
        ),

        # ── Motor node ─────────────────────────────────────────────────────────
        Node(
            package="waverower",
            executable="waverower",
            name="motor_hat_node",
            output="screen",
            parameters=[{
                "control_mode":       "manual",
                "i2c_bus":            LaunchConfiguration("i2c_bus"),
                "i2c_address":        LaunchConfiguration("i2c_address"),
                "manual_twist_topic": motor_twist_topic,
                # IMU PID korekcia – aktívna iba ak correction_mode=imu
                "imu_correction":     use_imu_correction,
                "imu_yaw_kp":         0.15,
                "imu_yaw_ki":         0.05,
                "imu_yaw_kd":         0.01,
                "imu_yaw_deadband":   0.02,
                "imu_yaw_integral_limit": 0.3,
            }],
        ),

        # ── Optical flow korekcia (len ak correction_mode=optical_flow) ────────
        Node(
            package="waverower",
            executable="optical_flow",
            name="optical_flow_node",
            output="screen",
            condition=IfCondition(use_flow_correction),
            parameters=[{
                "correction_gain":   1.5,
                "max_correction":    0.3,
                "forward_threshold": 0.05,
                "steer_deadzone":    0.12,
                "min_features":      15,
            }],
        ),

        # ── Camera (potrebná pre optical_flow) ─────────────────────────────────
        Node(
            package="camera_ros",
            executable="camera_node",
            name="camera_node",
            namespace="camera",
            output="screen",
            condition=IfCondition(use_camera),
            parameters=[{"camera": "0"}],
            remappings=[
                ("image_raw",    "/camera/image_raw"),
                ("camera_info",  "/camera/camera_info"),
            ],
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
        # odom → base_link (identita – bez enkodérov, slam_toolbox koriguje map→odom)
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="odom_to_base_link",
            arguments=["0", "0", "0", "0", "0", "0", "odom", "base_link"],
        ),
        # base_link → imu_link
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

        # ── EKF: IMU → odom TF (voliteľný, slam beží aj bez neho) ───────────
        # slam.yaml má odom_frame=base_link, takže slam_toolbox funguje
        # aj bez EKF – publishuje map→base_link priamo.
        # EKF zapni len ak chceš IMU fúziu do odom: use_ekf:=true
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            condition=IfCondition(LaunchConfiguration("use_ekf")),
            parameters=[ekf_params],
        ),

        # ── SLAM Toolbox (online async, lifecycle node) ───────────────────────
        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[slam_params],
        ),
        # Lifecycle: configure po 2s, activate po 4s
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

        # ── RViz2 (voliteľný, len ak use_rviz:=true) ──────────────────────────
        # Zvyčajne sa spúšťa na PC cez ROS_DOMAIN_ID + ROS_DISCOVERY_SERVER.
        # Ak chceš RViz lokálne na RPi: use_rviz:=true
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            condition=IfCondition(use_rviz),
            arguments=["-d", os.path.join(pkg, "params", "slam.rviz")],
        ),
    ])
