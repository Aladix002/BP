#!/usr/bin/env python3
"""Manuálna jazda (waverower) + kamera (camera_ros) + voliteľne IMU (Arduino USB fusion), LD19, teleop.

Zarovnanie pri jazde rovno (predvolene IMU):
  correction_mode:=imu           – gyro PID v motore (predvolené)
  correction_mode:=optical_flow  – Lucas-Kanade / Farnebäck z kamery (vypne IMU PID)
  correction_mode:=none          – bez korekcie

Optical flow vyžaduje use_camera:=true a komprimovaný obraz z camera_ros.
"""

import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    use_camera = LaunchConfiguration("use_camera")
    camera_id = LaunchConfiguration("camera_id")
    use_imu    = LaunchConfiguration("use_imu")
    use_imu_kalman = LaunchConfiguration("use_imu_kalman")
    use_lidar  = LaunchConfiguration("use_lidar")
    use_teleop = LaunchConfiguration("use_teleop")
    correction_mode = LaunchConfiguration("correction_mode")
    flow_algo  = LaunchConfiguration("flow_algo")
    use_ball_follow = LaunchConfiguration("use_ball_follow")

    ldlidar_share = get_package_share_directory("ldlidar_ros2")
    ld19_launch = os.path.join(ldlidar_share, "launch", "ld19.launch.py")

    lidar_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ld19_launch),
        condition=IfCondition(use_lidar),
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
                parameters=[{"camera": camera_id}],
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

    use_imu_correction = PythonExpression(['"', correction_mode, '" == "imu"'])

    # optical_flow → /teleop_cmd_vel_corrected; imu | none → /teleop_cmd_vel
    motor_twist_topic = PythonExpression([
        '"/teleop_cmd_vel_corrected" if "', correction_mode, '" == "optical_flow"',
        ' else "/teleop_cmd_vel"',
    ])

    return LaunchDescription(
        [
            SetEnvironmentVariable(name="ROS_DOMAIN_ID", value="0"),
            DeclareLaunchArgument(
                "use_camera",
                default_value="false",
                description="Spusti camera_ros (camera_node v /camera); topic napr. /camera/image_raw.",
            ),
            DeclareLaunchArgument(
                "camera_id",
                default_value="0",
                description="Parameter camera pre camera_ros: index 0,1,... alebo napr. /dev/video0 (Záleží od ovládača).",
            ),
            DeclareLaunchArgument(
                "use_imu",
                default_value="false",
                description="Arduino USB: imu_serial_fusion_bridge (CSV 15/20 polí) → /imu.",
            ),
            DeclareLaunchArgument("imu_serial_port", default_value="/dev/serial/by-id/usb-Arduino_Nano_R4_3501110A36313236694133344B573230-if00"),
            DeclareLaunchArgument("imu_baud_rate", default_value="115200"),
            DeclareLaunchArgument("imu_frame_id", default_value="imu_link"),
            DeclareLaunchArgument(
                "use_imu_kalman",
                default_value="false",
                description="imu_kalman_filter: /imu → /imu/filtered (6× 1D Kalman na a, ω).",
            ),
            DeclareLaunchArgument(
                "imu_kalman_output",
                default_value="/imu/filtered",
                description="Výstup vyhladeného Imu.",
            ),
            DeclareLaunchArgument(
                "use_lidar",
                default_value="false",
                description="Include ldlidar_ros2 ld19.launch.py (scan na /scan).",
            ),
            DeclareLaunchArgument(
                "use_teleop",
                default_value="false",
                description=(
                    "teleop_twist_keyboard → /teleop_cmd_vel. Vyžaduje TTY; v launch je emulate_tty. "
                    "Ak stále padá (Cursor/SSH), spusti teleop v druhom termináli: "
                    "ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/teleop_cmd_vel"
                ),
            ),
            DeclareLaunchArgument(
                "correction_mode",
                default_value="imu",
                description="Zarovnanie jazdy rovno: imu | optical_flow | none (predvolene imu).",
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
            DeclareLaunchArgument(
                "use_ball_follow",
                default_value="false",
                description="Spusti ball_follower (vyžaduje use_camera:=true).",
            ),
            Node(
                package="waverower",
                executable="waverower_motor",
                name="motor_hat_node",
                output="screen",
                parameters=[{
                    "control_mode":       LaunchConfiguration("control_mode"),
                    "i2c_bus":            LaunchConfiguration("i2c_bus"),
                    "i2c_address":        LaunchConfiguration("i2c_address"),
                    "manual_twist_topic": motor_twist_topic,
                    "imu_correction":     use_imu_correction,
                    "imu_yaw_kp":         0.15,
                    "imu_yaw_ki":         0.05,
                    "imu_yaw_kd":         0.01,
                    "imu_yaw_deadband":   0.02,
                    "imu_yaw_integral_limit": 0.3,
                }],
            ),
            Node(
                package="waverower",
                executable="ball_follower_action.py",
                name="ball_follower",
                output="screen",
                condition=IfCondition(use_ball_follow),
                parameters=[{
                    "image_topic":    "/camera/image_raw",
                    "cmd_topic":      "/cmd_vel",
                    "invert_angular": True,
                }],
            ),
            Node(
                package="waverower",
                executable="optical_flow",
                name="optical_flow_node",
                output="screen",
                condition=IfCondition(
                    PythonExpression([
                        '"', correction_mode, '" == "optical_flow" and "', flow_algo, '" != "farneback"',
                    ])
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
                    PythonExpression([
                        '"', correction_mode, '" == "optical_flow" and "', flow_algo, '" == "farneback"',
                    ])
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
                emulate_tty=True,
                remappings=[("cmd_vel", "/teleop_cmd_vel")],
                condition=IfCondition(use_teleop),
            ),
            Node(
                package="waverower",
                executable="imu_serial_fusion_bridge.py",
                name="imu_serial_fusion_bridge",
                output="screen",
                condition=IfCondition(use_imu),
                parameters=[
                    {
                        "serial_port": LaunchConfiguration("imu_serial_port"),
                        "baud_rate": LaunchConfiguration("imu_baud_rate"),
                        "frame_id": LaunchConfiguration("imu_frame_id"),
                        "topic": "/imu",
                    }
                ],
            ),
            Node(
                package="waverower",
                executable="imu_kalman_filter.py",
                name="imu_kalman_filter",
                output="screen",
                condition=IfCondition(use_imu_kalman),
                parameters=[
                    {
                        "input_topic": "/imu",
                        "output_topic": LaunchConfiguration("imu_kalman_output"),
                    }
                ],
            ),
            lidar_include,
            *camera_stack,
        ]
    )
