#!/usr/bin/env python3
# Bringup: motor (I2C HAT), volitelne kamera/IMU/LiDAR/teleop.
# SLAM je v runtime_stack.launch.py (plati pre manual aj wander).

import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Rozlisenie kamery musi sediet s ball follow / runtime_stack (jeden zdroj pravdy pre obraz)
CAMERA_WIDTH = 320
CAMERA_HEIGHT = 240
CAMERA_FORMAT = "XRGB8888"


def generate_launch_description():
    # LaunchConfiguration sa pouziva v Node/conditions; hodnoty sa beru pri starte
    use_camera = LaunchConfiguration("use_camera")
    camera_id = LaunchConfiguration("camera_id")
    use_robot_model = LaunchConfiguration("use_robot_model")
    robot_model_file = LaunchConfiguration("robot_model_file")
    use_imu    = LaunchConfiguration("use_imu")
    use_imu_kalman = LaunchConfiguration("use_imu_kalman")
    use_lidar  = LaunchConfiguration("use_lidar")
    use_teleop = LaunchConfiguration("use_teleop")
    correction_mode = LaunchConfiguration("correction_mode")
    use_cmd_vel_odom = LaunchConfiguration("use_cmd_vel_odom")
    teleop_max_linear = LaunchConfiguration("teleop_max_linear")
    teleop_max_angular = LaunchConfiguration("teleop_max_angular")
    control_mode = LaunchConfiguration("control_mode")

    # LD19 driver z balika ldlidar_ros2 (USB seriova linka nastavena v ich ld19.launch.py)
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
        # camera_node v namespace /camera; remap na globalne topic /camera/image_raw (jednoduchsie pre detekciu/web)
        camera_stack = [
            Node(
                package="camera_ros",
                executable="camera_node",
                name="camera_node",
                namespace="camera",
                output="screen",
                condition=IfCondition(use_camera),
                parameters=[{
                    "camera": camera_id,
                    "width": CAMERA_WIDTH,
                    "height": CAMERA_HEIGHT,
                    "format": CAMERA_FORMAT,
                }],
                remappings=[
                    ("image_raw", "/camera/image_raw"),
                    ("camera_info", "/camera/camera_info"),
                ],
            )
        ]
    else:
        # Bez camera_ros netreba spustat uzol; ak user chce kameru, vypiseme co nainstalovat
        camera_stack = [
            LogInfo(
                condition=IfCondition(use_camera),
                msg=(
                    "use_camera:=true vyzaduje nainstalovany balik camera_ros "
                    "(napr. sudo apt install ros-jazzy-camera-ros)."
                ),
            )
        ]

    try:
        waver_sim_share = get_package_share_directory("waver_sim")
        have_waver_sim = True
    except PackageNotFoundError:
        waver_sim_share = ""
        have_waver_sim = False

    robot_model_stack = []
    if have_waver_sim:
        # URDF z Gazebo balika: robot_state_publisher posiela joint_state -> TF pre vizualizaciu
        default_robot_model = os.path.join(waver_sim_share, "description", "robot.urdf.xacro")
        robot_description = ParameterValue(
            Command(["xacro ", robot_model_file]),
            value_type=str,
        )
        robot_model_stack = [
            DeclareLaunchArgument(
                "robot_model_file",
                default_value=default_robot_model,
                description="cesta k URDF/Xacro modelu robota pre robot_state_publisher",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                condition=IfCondition(use_robot_model),
                parameters=[{
                    "robot_description": robot_description,
                }],
            ),
        ]
    else:
        robot_model_stack = [
            DeclareLaunchArgument(
                "robot_model_file",
                default_value="",
                description="cesta k URDF/Xacro modelu robota pre robot_state_publisher",
            ),
            LogInfo(
                condition=IfCondition(use_robot_model),
                msg=(
                    "use_robot_model:=true vyzaduje nainstalovany balik waver_sim "
                    "(obsahuje robot.urdf.xacro)."
                ),
            ),
        ]

    # PythonExpression: stringova kontrola parametru (correction_mode nie je vzdy C++ enum)
    use_imu_correction = PythonExpression(['"', correction_mode, '" == "imu"'])

    motor_twist_topic = "/teleop_cmd_vel"

    return LaunchDescription(
        [
            # Izolovat DDS traffic v LAN (default 0; rovnake cislo na RPi a PC s RViz)
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
                "use_robot_model",
                default_value="true",
                description="spusti robot_state_publisher a publikuj robot_description",
            ),
            DeclareLaunchArgument(
                "use_imu",
                default_value="true",
                description="Arduino USB -> imu_serial_fusion_bridge (CSV 15/20 poli) -> /imu",
            ),
            DeclareLaunchArgument(
                "use_cmd_vel_odom",
                default_value="true",
                description="fallback odometria z /teleop_cmd_vel (odom->base_link) pre RViz pohyb modelu",
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
                description="ldlidar_ros2 ld19.launch.py (/scan)",
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
                    "control_mode":        control_mode,
                    "i2c_bus":             LaunchConfiguration("i2c_bus"),
                    "i2c_address":         LaunchConfiguration("i2c_address"),
                    # pwm_min: prah kedy H-bridge zacne hybat motorom (anti-cvakanie); pwm_max plny vykon PCA9685
                    "pwm_min":             400,
                    "pwm_max":             4095,
                    # teleop posiela "rychlost" v m/s zmysle az po deleni touto konstantou -> normalizacia na [-1,1]
                    "teleop_max_linear":   teleop_max_linear,
                    "teleop_max_angular":  teleop_max_angular,
                    "invert_linear":       True,
                    "cmd_vel_invert_linear": True,
                    # wheel_base tu sluzi ako skala pre diferencial v cmd_vel vetve (zhoda s YAML ball follow)
                    "wheel_base":          2.0,
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
                executable="cmd_vel_odom.py",
                name="cmd_vel_odom",
                output="screen",
                condition=IfCondition(use_cmd_vel_odom),
                parameters=[
                    {
                        "cmd_topic":      "/teleop_cmd_vel",
                        "cmd_topic_auto": "/cmd_vel",
                        "odom_topic":     "/odom",
                        "odom_frame":     "odom",
                        "base_frame":     "base_link",
                        "publish_tf":     True,
                        "timeout_sec":    0.4,
                        "update_rate_hz": 30.0,
                        "use_imu_yaw":    True,
                        "imu_topic":      "/imu",
                        "linear_scale":   0.4,
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
            *robot_model_stack,
        ]
    )
