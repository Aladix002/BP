#!/bin/bash
# Spusti WaveRover simuláciu (Gazebo + SLAM + Nav2 + 1x RViz)
# Pouzitie: ./run_sim.sh

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "==> Cistim stare ROS2/Gazebo procesy..."
pkill -f "gz sim"                2>/dev/null || true
pkill -f "ros2 launch waver_nav" 2>/dev/null || true
pkill -f "ros2 launch waver_gazebo" 2>/dev/null || true
pkill -f "component_container_isolated" 2>/dev/null || true
pkill -f "slam_toolbox"          2>/dev/null || true
pkill -f "nav2_"                 2>/dev/null || true
pkill -f "parameter_bridge"      2>/dev/null || true
pkill -f "robot_state_publisher" 2>/dev/null || true
pkill -f "twist_mux"             2>/dev/null || true
pkill -f "rviz2"                 2>/dev/null || true
sleep 1

# Cistenie stale FastDDS SHM suborov (odstranuje RTPS_TRANSPORT_SHM warningy)
rm -f /dev/shm/fastrtps_* 2>/dev/null || true

# Zmensenie FastDDS problemov so shared-memory lockmi.
export RMW_FASTRTPS_USE_SHM=0
export ROS_DOMAIN_ID=0

echo "==> Source workspace..."
# Remove stale overlays from parent workspace shells.
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH ROS_PACKAGE_PATH
source /opt/ros/jazzy/setup.bash

if [ ! -f "$SCRIPT_DIR/install/setup.bash" ]; then
  echo "[WARN] install/setup.bash not found in gazebo workspace, building..."
  (cd "$SCRIPT_DIR" && colcon build --symlink-install)
fi

source "$SCRIPT_DIR/install/setup.bash"

echo "==> Spustam teleop v novom terminali..."
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
echo "[WARN] Nedá sa otvoriť terminál. Spusti manuálne:"
echo "  ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel_key"

if [ -d "$SCRIPT_DIR/waver_nav" ]; then
  echo "==> Spustam: ros2 launch waver_nav full_bringup.launch.py"
  ros2 launch waver_nav full_bringup.launch.py
else
  echo "[INFO] Balik waver_nav v zdrojoch neexistuje, pouzivam waver_sim."
  echo "==> Spustam: ros2 launch waver_sim launch_sim.launch.py"
  ros2 launch waver_sim launch_sim.launch.py
fi
