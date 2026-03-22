"""
bringup.launch.py  –  Wave Rover base bringup (sensors + control + robot model)

Starts:
  - robot_state_publisher  (publishes URDF → /robot_description for RViz)
  - joint_state_publisher  (publishes wheel joint states)
  - imu_node               (sensor: IMU via HTTP or I2C)
  - camera_node            (sensor: USB/CSI camera, optional)
  - watchdog_node          (safety: zero-vel on timeout)
  - imu_stabilizer_node    (control: angular PID correction)
  - mode_manager_node      (control: MANUAL/AUTO gating)
  - motor_controller_node  (control: Twist → PWM)

Usage:
  ros2 launch wave_rover_bringup bringup.launch.py
  ros2 launch wave_rover_bringup bringup.launch.py use_camera:=false use_mock:=true
  ros2 launch wave_rover_bringup bringup.launch.py use_imu_stabilizer:=false
"""

import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _get_robot_description() -> str:
    """Run xacro on waver_description URDF and return the XML string."""
    try:
        desc_pkg = get_package_share_directory('waver_description')
        xacro_file = os.path.join(desc_pkg, 'urdf', 'waver.xacro')
        result = subprocess.run(
            ['xacro', xacro_file, 'sim_control:=ros2', 'camera_type:=raspi'],
            capture_output=True, text=True, check=True,
        )
        return result.stdout
    except Exception as e:
        # waver_description not sourced – return a minimal URDF so RSP doesn't crash
        return (
            '<?xml version="1.0"?>'
            '<robot name="wave_rover">'
            '<link name="base_link"/>'
            '</robot>'
        )


def generate_launch_description():
    pkg = get_package_share_directory('wave_rover_bringup')
    params_file = os.path.join(pkg, 'config', 'robot_params.yaml')
    robot_description = _get_robot_description()

    # ── arguments ─────────────────────────────────────────────────────────
    args = [
        DeclareLaunchArgument('use_camera',         default_value='false'),
        DeclareLaunchArgument('use_imu_stabilizer', default_value='true'),
        DeclareLaunchArgument('use_mock',            default_value='false',
                              description='Use mock motor driver (no hardware)'),
        DeclareLaunchArgument('initial_mode',        default_value='manual',
                              description='"manual" or "auto"'),
        DeclareLaunchArgument('esp32_ip',            default_value='192.168.0.224'),
        DeclareLaunchArgument('imu_backend',         default_value='http',
                              description='"http" (ESP32) or "i2c"'),
    ]

    use_camera         = LaunchConfiguration('use_camera')
    use_imu_stabilizer = LaunchConfiguration('use_imu_stabilizer')
    use_mock           = LaunchConfiguration('use_mock')
    initial_mode       = LaunchConfiguration('initial_mode')
    esp32_ip           = LaunchConfiguration('esp32_ip')
    imu_backend        = LaunchConfiguration('imu_backend')

    # ── nodes ────────────────────────────────────────────────────────────

    # Robot model – publishes /robot_description so RViz can show the 3-D model
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': False}],
    )

    # Publishes wheel joint positions (needed for the robot model to render correctly)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
    )

    imu_node = Node(
        package='wave_rover_sensors',
        executable='imu_node',
        name='imu_node',
        output='screen',
        parameters=[params_file, {
            'backend':  imu_backend,
            'esp32_ip': esp32_ip,
        }],
    )

    camera_node = Node(
        package='wave_rover_sensors',
        executable='camera_node',
        name='camera_node',
        output='screen',
        parameters=[params_file],
        condition=IfCondition(use_camera),
    )

    watchdog_node = Node(
        package='wave_rover_control',
        executable='watchdog',
        name='watchdog_node',
        output='screen',
        parameters=[params_file],
        remappings=[
            ('cmd_vel',      'cmd_vel_raw'),
            ('cmd_vel_safe', 'cmd_vel'),
        ],
    )

    imu_stabilizer_node = Node(
        package='wave_rover_control',
        executable='imu_stabilizer',
        name='imu_stabilizer_node',
        output='screen',
        parameters=[params_file],
        condition=IfCondition(use_imu_stabilizer),
    )

    web_server = Node(
        package='wave_rover_sensors',
        executable='web_server',
        name='web_server_node',
        output='screen',
        parameters=[{'port': 8888}],
    )

    rosbridge = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[{'port': 9090}],
    )

    node_manager = Node(
        package='wave_rover_control',
        executable='node_manager',
        name='node_manager_node',
        output='screen',
        parameters=[params_file],
    )

    shutdown_node = Node(
        package='wave_rover_control',
        executable='shutdown_node',
        name='shutdown_node',
        output='screen',
        parameters=[params_file],
    )

    mode_manager_node = Node(
        package='wave_rover_control',
        executable='mode_manager',
        name='mode_manager_node',
        output='screen',
        parameters=[params_file, {'initial_mode': initial_mode}],
    )

    motor_controller_node = Node(
        package='wave_rover_control',
        executable='motor_controller',
        name='motor_controller_node',
        output='screen',
        parameters=[params_file, {'use_mock': use_mock}],
    )

    return LaunchDescription([
        *args,
        LogInfo(msg='Wave Rover bringup starting...'),
        robot_state_publisher,
        joint_state_publisher,
        imu_node,
        camera_node,
        watchdog_node,
        imu_stabilizer_node,
        mode_manager_node,
        web_server,
        rosbridge,
        node_manager,
        shutdown_node,
        motor_controller_node,
        LogInfo(msg='Wave Rover bringup complete.  '
                    'Publish to /mode (std_msgs/String: "manual"/"auto") to switch modes.'),
    ])
