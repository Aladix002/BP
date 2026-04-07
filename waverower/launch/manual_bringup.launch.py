#!/usr/bin/env python3
"""Manualna jazda + kamera + volitelne IMU, LD19, teleop; predvolene SLAM (slam_toolbox).

correction_mode: imu | none
SLAM: use_slam:=true a use_lidar:=true -> /map. Vypnut: use_slam:=false.

Ulozenie mapy: ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '/cesta/mapa'}}"

"""

import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _slam_stack(context, *args, **kwargs):
    use_slam = LaunchConfiguration("use_slam").perform(context) == "true"
    use_lidar = LaunchConfiguration("use_lidar").perform(context) == "true"
    if not use_slam:
        return []
    pkg = get_package_share_directory("waverower")
    slam_params = os.path.join(pkg, "params", "slam.yaml")
    ekf_params = os.path.join(pkg, "params", "ekf.yaml")
    use_ekf = LaunchConfiguration("use_ekf").perform(context) == "true"
    use_rviz = LaunchConfiguration("use_rviz").perform(context) == "true"
    if not use_lidar:
        return [
            LogInfo(
                msg=(
                    "use_slam:=true ale use_lidar:=false - SLAM sa nespusta (chyba /scan). "
                    "Nastav use_lidar:=true."
                ),
            ),
        ]
    actions = []
    if not use_ekf:
        actions.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="odom_to_base_link",
                output="screen",
                arguments=["0", "0", "0", "0", "0", "0", "odom", "base_link"],
            )
        )
    actions.append(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_link_to_imu",
            output="screen",
            arguments=["0", "0", "0.05", "0", "0", "0", "base_link", "imu_link"],
        )
    )
    if use_ekf:
        actions.append(
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                output="screen",
                parameters=[ekf_params],
            )
        )
    actions.extend(
        [
            Node(
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                output="screen",
                parameters=[slam_params],
            ),
            TimerAction(
                period=2.0,
                actions=[
                    ExecuteProcess(
                        cmd=["ros2", "lifecycle", "set", "/slam_toolbox", "configure"],
                        output="screen",
                    )
                ],
            ),
            TimerAction(
                period=4.0,
                actions=[
                    ExecuteProcess(
                        cmd=["ros2", "lifecycle", "set", "/slam_toolbox", "activate"],
                        output="screen",
                    )
                ],
            ),
        ]
    )
    if use_rviz:
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", os.path.join(pkg, "params", "slam.rviz")],
            )
        )
    return actions


def generate_launch_description():
    use_camera = LaunchConfiguration("use_camera")
    camera_id = LaunchConfiguration("camera_id")
    use_imu    = LaunchConfiguration("use_imu")
    use_imu_kalman = LaunchConfiguration("use_imu_kalman")
    use_lidar  = LaunchConfiguration("use_lidar")
    use_teleop = LaunchConfiguration("use_teleop")
    correction_mode = LaunchConfiguration("correction_mode")
    use_ball_follow = LaunchConfiguration("use_ball_follow")
    teleop_max_linear = LaunchConfiguration("teleop_max_linear")
    teleop_max_angular = LaunchConfiguration("teleop_max_angular")

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
                    "use_camera:=true vyzaduje nainstalovany balik camera_ros "
                    "(napr. sudo apt install ros-jazzy-camera-ros)."
                ),
            )
        ]

    use_imu_correction = PythonExpression(['"', correction_mode, '" == "imu"'])

    motor_twist_topic = "/teleop_cmd_vel"

    return LaunchDescription(
        [
            SetEnvironmentVariable(name="ROS_DOMAIN_ID", value="0"),
            DeclareLaunchArgument(
                "use_camera",
                default_value="false",
                description="camera_ros (camera_node v /camera), topic napr. /camera/image_raw",
            ),
            DeclareLaunchArgument(
                "camera_id",
                default_value="0",
                description="camera parameter pre camera_ros: index alebo /dev/video0",
            ),
            DeclareLaunchArgument(
                "use_imu",
                default_value="false",
                description="Arduino USB -> imu_serial_fusion_bridge (CSV 15/20 poli) -> /imu",
            ),
            DeclareLaunchArgument("imu_serial_port", default_value="/dev/serial/by-id/usb-Arduino_Nano_R4_3501110A36313236694133344B573230-if00"),
            DeclareLaunchArgument("imu_baud_rate", default_value="115200"),
            DeclareLaunchArgument("imu_frame_id", default_value="imu_link"),
            DeclareLaunchArgument(
                "use_imu_kalman",
                default_value="false",
                description="imu_kalman_filter: /imu -> /imu/filtered (6x 1D Kalman)",
            ),
            DeclareLaunchArgument(
                "imu_kalman_output",
                default_value="/imu/filtered",
                description="vystup vyhladeneho Imu",
            ),
            DeclareLaunchArgument(
                "use_lidar",
                default_value="true",
                description="ldlidar_ros2 ld19.launch.py (/scan), default true pre SLAM",
            ),
            DeclareLaunchArgument(
                "use_slam",
                default_value="true",
                description="slam_toolbox async + TF (vyzaduje use_lidar:=true)",
            ),
            DeclareLaunchArgument(
                "use_ekf",
                default_value="false",
                description="robot_localization EKF (IMU->odom); ak true, bez statickeho odom->base_link",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="false",
                description="RViz2 + slam.rviz (na RPi narocne; casto RViz na PC, rovnaky DOMAIN_ID)",
            ),
            DeclareLaunchArgument(
                "use_teleop",
                default_value="false",
                description=(
                    "teleop_twist_keyboard -> /teleop_cmd_vel. Vyzaduje TTY; launch ma emulate_tty. "
                    "Ak pada (Cursor/SSH), spusti teleop v druhom terminale: "
                    "ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/teleop_cmd_vel"
                ),
            ),
            DeclareLaunchArgument(
                "correction_mode",
                default_value="imu",
                description="zarovnanie rovno: imu | none",
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
                description="ball_follower (vyzaduje use_camera:=true)",
            ),
            DeclareLaunchArgument(
                "teleop_max_linear",
                default_value="1.0",
                description="max |linear.x| pri 100 % teleop / web UI (runtime_stack rovnake)",
            ),
            DeclareLaunchArgument(
                "teleop_max_angular",
                default_value="2.0",
                description="max |angular.z| pri 100 % teleop / web UI",
            ),
            Node(
                package="waverower",
                executable="waverower_motor",
                name="motor_hat_node",
                output="screen",
                parameters=[{
                    "control_mode":        LaunchConfiguration("control_mode"),
                    "i2c_bus":             LaunchConfiguration("i2c_bus"),
                    "i2c_address":         LaunchConfiguration("i2c_address"),
                    # PWM rozsah: pwm_min = minimum kedy sa motor pohne, pwm_max = maximum
                    "pwm_min":             400,
                    "pwm_max":             4095,
                    # Teleop normalizacia: teleop_max_linear = rychlost pri ktore motor ide naplno
                    # Teleop klaves 'i' posiela linear.x = speed (napr 0.5 az 3.0 podla nastavenia q/z)
                    # Nastav tuto hodnotu = maximalna rychlost ktoru chces pouzivat v teleop
                    "teleop_max_linear":   teleop_max_linear,
                    "teleop_max_angular":  teleop_max_angular,
                    "invert_linear":       True,
                    "cmd_vel_invert_linear": True,
                    "smooth_alpha":        0.20,
                    "imu_correction":      use_imu_correction,
                    "imu_kp":              0.30,
                    "imu_ki":              0.05,
                    "imu_kd":              0.01,
                    "imu_deadband":        0.02,
                    "imu_windup":          0.30,
                    "imu_sign":            -1.0,
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
            OpaqueFunction(function=_slam_stack),
        ]
    )
