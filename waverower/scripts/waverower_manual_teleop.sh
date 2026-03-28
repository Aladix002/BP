#!/usr/bin/env bash
# Spustí waverower (motor). Teleop:
#   • Lokálne: ak je DISPLAY/WAYLAND, otvorí nový terminál s teleop_twist_keyboard.
#   • RPi cez SSH / bez grafiky: len motor + návod na teleop na PC (rovnaká sieť, ROS_DOMAIN_ID).
#
# Prepínače:  --motor-only     vždy len motor (nepokúša sa lokálny teleop)
#
# Prostredie:  WAVEROWER_WS, ROS_DISTRO (voliteľné; ROS_DOMAIN_ID zdieľaj s PC)

# Bez nounset: ROS setup.bash používa nepovinné premenné (napr. AMENT_TRACE_SETUP_FILES).
set -eo pipefail
export ROS_DOMAIN_ID=0

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SELF="${SCRIPT_DIR}/$(basename "${BASH_SOURCE[0]}")"
export WAVEROWER_WS="${WAVEROWER_WS:-$HOME/Desktop/BP}"

if [ -z "${ROS_DISTRO:-}" ]; then
  for d in jazzy humble iron; do
    if [ -f "/opt/ros/$d/setup.bash" ]; then
      ROS_DISTRO=$d
      break
    fi
  done
fi
export ROS_DISTRO="${ROS_DISTRO:-jazzy}"

if [ "${1:-}" = "--teleop-subprocess" ]; then
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  if [ ! -f "${WAVEROWER_WS}/install/setup.bash" ]; then
    echo "Chýba ${WAVEROWER_WS}/install/setup.bash — v tomto workspace spusti: colcon build --packages-select waverower" >&2
    exit 1
  fi
  source "${WAVEROWER_WS}/install/setup.bash"
  ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/teleop_cmd_vel
  exec bash -l
fi

MOTOR_ONLY=0
for arg in "$@"; do
  if [ "$arg" = "--motor-only" ]; then
    MOTOR_ONLY=1
  fi
done

# --- hlavný beh: vyčistenie + motor + (voliteľne) lokálny teleop ---

echo "Ukončujem predchádzajúce waverower / teleop / manual_bringup..."
pkill -f 'manual_bringup[.]launch[.]py' 2>/dev/null || true
pkill -f 'lib/waverower/waverower' 2>/dev/null || true
pkill -f 'ros2 run waverower waverower' 2>/dev/null || true
pkill -f 'teleop_twist_keyboard' 2>/dev/null || true
sleep 0.4

source "/opt/ros/${ROS_DISTRO}/setup.bash"
if [ ! -f "${WAVEROWER_WS}/install/setup.bash" ]; then
  echo "Chýba ${WAVEROWER_WS}/install/setup.bash — spusti colcon build v ${WAVEROWER_WS}" >&2
  exit 1
fi
source "${WAVEROWER_WS}/install/setup.bash"

cleanup() {
  pkill -f 'teleop_twist_keyboard' 2>/dev/null || true
  if [ -n "${WAVER_PID:-}" ]; then
    kill "$WAVER_PID" 2>/dev/null || true
    wait "$WAVER_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

_print_pc_teleop_help() {
  local dom="${ROS_DOMAIN_ID:-0}"
  echo ""
  echo "=== Teleop na osobnom PC (DDS: rovnaká LAN, firewall, ROS_DOMAIN_ID) ==="
  echo "Na tomto stroji je ROS_DOMAIN_ID=${dom} (ak ho nenastavíš, oba konce často používajú 0)."
  echo "Na PC otvor terminál a spusti (uprav distro, ak nemáš jazzy):"
  echo ""
  echo "  export ROS_DOMAIN_ID=${dom}"
  echo "  source /opt/ros/${ROS_DISTRO}/setup.bash"
  echo "  ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/teleop_cmd_vel"
  echo ""
  echo "Overenie zo PC:  ros2 topic list   (mali by sa objaviť uzly/témy z Raspberry Pi)."
  echo "Ak nič nevidíš: rovnaká Wi‑Fi/sieť, skontroluj firewall (UDP multicast), RMW (FastDDS/Cyclone)."
  echo ""
}

echo "Spúšťam waverower (tento terminál = logy motora)."
ros2 run waverower waverower &
WAVER_PID=$!
sleep 0.7

_open_teleop_terminal() {
  export WAVEROWER_WS ROS_DISTRO
  if command -v gnome-terminal >/dev/null 2>&1; then
    gnome-terminal --title='Waverower teleop' -- bash -c "export WAVEROWER_WS=\"$WAVEROWER_WS\" ROS_DISTRO=\"$ROS_DISTRO\"; bash \"$SELF\" --teleop-subprocess"
    return 0
  fi
  if command -v xfce4-terminal >/dev/null 2>&1; then
    xfce4-terminal --title='Waverower teleop' -e "bash -c 'export WAVEROWER_WS=\"$WAVEROWER_WS\" ROS_DISTRO=\"$ROS_DISTRO\"; bash \"$SELF\" --teleop-subprocess'" &
    return 0
  fi
  if command -v lxterminal >/dev/null 2>&1; then
    lxterminal --title='Waverower teleop' -e "bash -c 'export WAVEROWER_WS=\"$WAVEROWER_WS\" ROS_DISTRO=\"$ROS_DISTRO\"; bash \"$SELF\" --teleop-subprocess'" &
    return 0
  fi
  if command -v konsole >/dev/null 2>&1; then
    konsole --title 'Waverower teleop' -e bash -c "export WAVEROWER_WS=\"$WAVEROWER_WS\" ROS_DISTRO=\"$ROS_DISTRO\"; bash \"$SELF\" --teleop-subprocess" &
    return 0
  fi
  if command -v xterm >/dev/null 2>&1; then
    xterm -title 'Waverower teleop' -e bash -c "export WAVEROWER_WS=\"$WAVEROWER_WS\" ROS_DISTRO=\"$ROS_DISTRO\"; bash \"$SELF\" --teleop-subprocess" &
    return 0
  fi
  return 1
}

HAS_GUI=0
if [ -n "${DISPLAY:-}" ] || [ -n "${WAYLAND_DISPLAY:-}" ]; then
  HAS_GUI=1
fi

if [ "$MOTOR_ONLY" = 1 ]; then
  echo "Režim --motor-only: Ctrl+C tu zastaví len motor (teleop máš kde inde)."
  _print_pc_teleop_help
elif [ "$HAS_GUI" = 1 ]; then
  echo "Ctrl+C tu zastaví motor aj lokálny teleop (ak beží na tomto stroji)."
  if _open_teleop_terminal; then
    echo "Teleop je v novom okne terminálu (klikni doň a ovládaj šípkami / podľa nápovedy)."
  else
    echo "Nenašiel sa grafický terminál — teleop na tomto stroji spusti:" >&2
    echo "  WAVEROWER_WS=\"$WAVEROWER_WS\" ROS_DISTRO=\"$ROS_DISTRO\" \"$SELF\" --teleop-subprocess" >&2
    _print_pc_teleop_help
  fi
else
  echo "Bez DISPLAY (napr. SSH na RPi): teleop spúšťaj na PC — Ctrl+C tu zastaví len motor."
  _print_pc_teleop_help
fi

wait "$WAVER_PID"
