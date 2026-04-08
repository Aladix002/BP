#!/usr/bin/env python3
# Bringup: motor (I2C HAT), volitelne kamera/IMU/LiDAR/teleop, SLAM (slam_toolbox).
# Mapa: ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '/cesta/mapa'}}"

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
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Rozlisenie kamery musi sediet s ball follow / runtime_stack (jeden zdroj pravdy pre obraz)
CAMERA_WIDTH = 320
CAMERA_HEIGHT = 240
CAMERA_FORMAT = "XRGB8888"


def _slam_stack(context, *args, **kwargs):
    # Vyhodnotenie argumentov az pri spusteni launchu (nie skor), aby sedeli override z CLI
    use_slam = LaunchConfiguration("use_slam").perform(context) == "true"
    use_lidar = LaunchConfiguration("use_lidar").perform(context) == "true"
    use_cmd_vel_odom = LaunchConfiguration("use_cmd_vel_odom").perform(context) == "true"
    if not use_slam:
        # Bez SLAM nic z tejto funkcie nepridavame (ziadny slam_toolbox, ziadna extra TF logika)
        return []
    pkg = get_package_share_directory("waverower")
    slam_params = os.path.join(pkg, "params", "slam.yaml")
    ekf_params = os.path.join(pkg, "params", "ekf.yaml")
    use_ekf = LaunchConfiguration("use_ekf").perform(context) == "true"
    use_rviz = LaunchConfiguration("use_rviz").perform(context) == "true"
    if not use_lidar:
        # SLAM z LiDAR scanu: bez /scan nema zmysel async_slam_toolbox spustat
        return [
            LogInfo(
                msg=(
                    "use_slam:=true ale use_lidar:=false - SLAM sa nespusta (chyba /scan). "
                    "Nastav use_lidar:=true."
                ),
            ),
        ]
    actions = []
    if not use_ekf and not use_cmd_vel_odom:
        # Ak nemame ani EKF ani integrator z teleopu, TF strom potrebuje aspon pevny odom->base_link,
        # inak RViz/model nemaju vztah medzi mapou a robotom (slam_toolbox mapuje scan do map->odom)
        actions.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="odom_to_base_link",
                output="screen",
                arguments=["0", "0", "0", "0", "0", "0", "odom", "base_link"],
            )
        )
    # IMU je fyzicky nad podvozkom: static TF len posun Z (5 cm), rotacia 0 (ak nemas kalibraciu nastrojov)
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
        # EKF fuzuje IMU (a prip. ine senzory podla ekf.yaml) do odom; nahradza staticky odom->base_link vyssie
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
            # async_slam_toolbox: ziaden synchronny spin; mapuje sa na pozadi (vhodne pre RPi)
            Node(
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                output="screen",
                parameters=[slam_params],
            ),
            # Lifecycle: node je najprv neaktivny; bez configure/activate neprijima ciele
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
        # RViz na RPi je tazky; casto sa spusta na PC so rovnakym ROS_DOMAIN_ID
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
                default_value="false",
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
                        # Integracia /teleop_cmd_vel -> fiktivna odometria (enkodery nemame)
                        "cmd_topic": "/teleop_cmd_vel",
                        "odom_topic": "/odom",
                        "odom_frame": "odom",
                        "base_frame": "base_link",
                        "publish_tf": True,
                        "timeout_sec": 0.4,
                        "update_rate_hz": 30.0,
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
            OpaqueFunction(function=_slam_stack),
        ]
    )
