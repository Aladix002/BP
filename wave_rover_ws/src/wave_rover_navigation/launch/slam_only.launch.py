"""
slam_only.launch.py  –  SLAM Toolbox + EKF only (no Nav2)

Started dynamically by node_manager_node when nav_mode = "slam".
Do NOT run this directly with full_system.launch.py active.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav_pkg = get_package_share_directory('wave_rover_navigation')

    sim_arg = DeclareLaunchArgument('use_sim_time', default_value='false')
    use_sim_time = LaunchConfiguration('use_sim_time')

    slam_params = os.path.join(nav_pkg, 'config', 'slam_params.yaml')
    ekf_params  = os.path.join(nav_pkg, 'config', 'ekf_params.yaml')

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params, {'use_sim_time': use_sim_time}],
    )

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params, {'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([sim_arg, ekf_node, slam_node])
