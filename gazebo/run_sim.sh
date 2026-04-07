#!/bin/bash
# Pouzitie:
#   ./run_sim.sh

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Cistenie je dolezite, aby neostali stare nody s rovnakymi topicmi.
echo "==> Cistim stare ROS2/Gazebo procesy..."
pkill -f "gz sim"                2>/dev/null || true
pkill -f "ros2 launch waver_sim" 2>/dev/null || true
pkill -f "component_container_isolated" 2>/dev/null || true
pkill -f "slam_toolbox"          2>/dev/null || true
pkill -f "nav2_"                 2>/dev/null || true
pkill -f "parameter_bridge"      2>/dev/null || true
pkill -f "robot_state_publisher" 2>/dev/null || true
pkill -f "twist_mux"             2>/dev/null || true
pkill -f "rviz2"                 2>/dev/null || true
sleep 1

# FastDDS vie po pade nechat SHM lock subory; odstranenie predchadza warningom.
rm -f /dev/shm/fastrtps_* 2>/dev/null || true

# Vynuti DDS bez shared memory transportu (stabilnejsie pri opakovanom spustani).
export RMW_FASTRTPS_USE_SHM=0
# ROS_DOMAIN_ID drzi simulaciu v jednej DDS domene.
export ROS_DOMAIN_ID=0

echo "==> Source workspace..."
# Vymaze stare overlay cesty, aby sa nenacitaval iny workspace.
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH ROS_PACKAGE_PATH
source /opt/ros/jazzy/setup.bash

# Ak workspace nie je buildnuty, buildne sa automaticky.
if [ ! -f "$SCRIPT_DIR/install/setup.bash" ]; then
  echo "[WARN] install/setup.bash not found in gazebo workspace, building..."
  (cd "$SCRIPT_DIR" && colcon build --symlink-install)
fi

# Nacitanie lokalneho workspace (baliky waver_sim a waver_gazebo).
source "$SCRIPT_DIR/install/setup.bash"

echo "==> Spustam teleop v novom terminali..."
# Teleop bezi bokom: pouzivatel moze klavesami prebit Nav2.
gnome-terminal --title="Teleop (WASD)" -- bash -c "
  source /opt/ros/jazzy/setup.bash
  source '$SCRIPT_DIR/install/setup.bash'
  echo '=== TELEOP ==='
  echo 'WASD / sipky = pohyb   |   q/z = rychlost   |   Ctrl+C = stop'
  echo 'Klaves drz stlaceny. Pustenie na 0.5s -> Nav2 prebera riadenie.'
  echo ''
  ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -r /cmd_vel:=/cmd_vel_key
" 2>/dev/null || \
xterm -title "Teleop (WASD)" -e bash -c "
  source /opt/ros/jazzy/setup.bash
  source '$SCRIPT_DIR/install/setup.bash'
  echo '=== TELEOP ==='
  ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -r /cmd_vel:=/cmd_vel_key
  read
" 2>/dev/null || \
echo "[WARN] Neda sa otvorit terminal. Spusti manualne:"
echo "  ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel_key"

# Hlavny vstup simulacie: Gazebo + bridge + Nav2 + RViz.
echo "==> Spustam: ros2 launch waver_sim launch_sim.launch.py"
ros2 launch waver_sim launch_sim.launch.py
