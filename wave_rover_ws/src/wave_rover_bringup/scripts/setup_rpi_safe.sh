#!/usr/bin/env bash
# =============================================================================
# setup_rpi_safe.sh  –  RPi filesystem safety setup for Wave Rover
#
# Protects against corruption when the RPi is powered off without proper shutdown.
# Run ONCE on the Raspberry Pi as root.  Then reboot.
#
# Two independent protection layers:
#   1. Overlay filesystem   – root becomes read-only, writes go to RAM tmpfs
#   2. Hardware watchdog    – RPi hardware WDT reboots if software hangs
#   3. Sudoers shutdown     – lets the ROS 2 shutdown_node power off cleanly
#   4. Systemd service      – auto-starts ROS 2 on boot
#
# Usage:
#   sudo bash setup_rpi_safe.sh [--overlay] [--watchdog] [--shutdown-sudoers]
#   sudo bash setup_rpi_safe.sh          # runs all three
# =============================================================================

set -euo pipefail
CYAN='\033[0;36m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
RED='\033[0;31m'; NC='\033[0m'
info()  { echo -e "${CYAN}[INFO]${NC}  $*"; }
ok()    { echo -e "${GREEN}[ OK ]${NC}  $*"; }
warn()  { echo -e "${YELLOW}[WARN]${NC}  $*"; }
die()   { echo -e "${RED}[FAIL]${NC}  $*"; exit 1; }

[[ $EUID -ne 0 ]] && die "Run as root: sudo bash $0"

DO_OVERLAY=${1:-all}
DO_WATCHDOG=${1:-all}
DO_SUDOERS=${1:-all}

# Parse flags
for arg in "$@"; do
  case $arg in
    --overlay)           DO_OVERLAY=yes; DO_WATCHDOG=no; DO_SUDOERS=no ;;
    --watchdog)          DO_OVERLAY=no;  DO_WATCHDOG=yes; DO_SUDOERS=no ;;
    --shutdown-sudoers)  DO_OVERLAY=no;  DO_WATCHDOG=no; DO_SUDOERS=yes ;;
  esac
done

# =============================================================================
# 1. OVERLAY FILESYSTEM  (read-only root)
# =============================================================================
setup_overlay() {
  info "Setting up overlay (read-only) filesystem..."

  # Check if raspi-config supports overlay FS
  if command -v raspi-config &>/dev/null; then
    info "Using raspi-config to enable overlay..."
    # Non-interactive: raspi-config nonint enable_overlayfs
    raspi-config nonint enable_overlayfs && ok "Overlay FS enabled via raspi-config" && return
  fi

  # Manual overlayfs setup (Ubuntu-based RPi image)
  info "Manual overlay setup..."

  # Install overlayroot package
  apt-get install -y overlayroot &>/dev/null

  # Configure /etc/overlayroot.conf
  cat > /etc/overlayroot.conf <<'EOF'
# Wave Rover overlay filesystem configuration
# root filesystem is read-only; all writes go to RAM tmpfs
overlayroot="tmpfs:swap=1,recurse=0"
overlayroot_cfgdisk="disabled"
EOF

  # Make /boot/cmdline.txt point to overlayroot
  # (Ubuntu RPi uses /boot/firmware/cmdline.txt)
  for CMDLINE in /boot/cmdline.txt /boot/firmware/cmdline.txt; do
    if [[ -f "$CMDLINE" ]]; then
      if ! grep -q "overlayroot" "$CMDLINE"; then
        sed -i 's/$/ overlayroot=tmpfs/' "$CMDLINE"
        ok "overlayroot added to $CMDLINE"
      else
        warn "overlayroot already in $CMDLINE"
      fi
    fi
  done

  ok "Overlay FS configured (read-only root after reboot)"
  warn "After reboot, filesystem is READ-ONLY."
  warn "To make permanent changes, temporarily disable overlay:"
  warn "  sudo overlayroot-chroot     # enter writable chroot"
  warn "  sudo raspi-config nonint disable_overlayfs  # or disable via raspi-config"
}

# =============================================================================
# 2. HARDWARE WATCHDOG
# =============================================================================
setup_watchdog() {
  info "Setting up RPi hardware watchdog..."

  # Load watchdog kernel module
  if ! lsmod | grep -q bcm2835_wdt; then
    modprobe bcm2835_wdt
    echo "bcm2835_wdt" >> /etc/modules
    ok "bcm2835_wdt module loaded and added to /etc/modules"
  else
    ok "bcm2835_wdt already loaded"
  fi

  # Configure systemd watchdog
  mkdir -p /etc/systemd/system.conf.d/
  cat > /etc/systemd/system.conf.d/watchdog.conf <<'EOF'
[Manager]
RuntimeWatchdogSec=15
ShutdownWatchdogSec=5min
EOF

  # Configure /etc/watchdog.conf
  apt-get install -y watchdog &>/dev/null || true
  cat > /etc/watchdog.conf <<'EOF'
# Hardware watchdog for Wave Rover RPi
watchdog-device        = /dev/watchdog
watchdog-timeout       = 15
interval               = 5
max-load-1             = 24
min-memory             = 1
realtime               = yes
priority               = 1
EOF

  systemctl enable watchdog 2>/dev/null || true
  systemctl start  watchdog 2>/dev/null || true

  ok "Hardware watchdog configured (reboots if system hangs for >15s)"
}

# =============================================================================
# 3. SUDOERS – allow shutdown_node to power off without password
# =============================================================================
setup_sudoers() {
  info "Configuring sudoers for ROS2 shutdown..."

  ROS_USER=${SUDO_USER:-$(logname 2>/dev/null || echo "pi")}

  cat > /etc/sudoers.d/ros_wave_rover <<EOF
# Allow Wave Rover ROS2 nodes to trigger safe shutdown
${ROS_USER} ALL=(ALL) NOPASSWD: /sbin/shutdown
${ROS_USER} ALL=(ALL) NOPASSWD: /sbin/poweroff
${ROS_USER} ALL=(ALL) NOPASSWD: /bin/systemctl poweroff
EOF
  chmod 440 /etc/sudoers.d/ros_wave_rover
  ok "Sudoers configured for user: ${ROS_USER}"
}

# =============================================================================
# 4. SYSTEMD SERVICE – auto-start ROS2 on boot
# =============================================================================
setup_autostart() {
  info "Creating wave_rover systemd service..."

  ROS_USER=${SUDO_USER:-$(logname 2>/dev/null || echo "pi")}
  HOME_DIR=$(getent passwd "$ROS_USER" | cut -d: -f6)

  cat > /etc/systemd/system/wave_rover.service <<EOF
[Unit]
Description=Wave Rover ROS2 System
After=network.target
Wants=network.target

[Service]
Type=simple
User=${ROS_USER}
Environment="HOME=${HOME_DIR}"
WorkingDirectory=${HOME_DIR}
ExecStartPre=/bin/sleep 5
ExecStart=/bin/bash -c 'source /opt/ros/jazzy/setup.bash && \
  source ${HOME_DIR}/Desktop/School/BPC-PRP/waver-humble/install/setup.bash && \
  source ${HOME_DIR}/Desktop/School/BPC-PRP/wave_rover_ws/install/setup.bash && \
  ros2 launch wave_rover_bringup full_system.launch.py use_rviz:=false'
Restart=on-failure
RestartSec=5
KillMode=mixed
TimeoutStopSec=10

# Systemd watchdog integration
WatchdogSec=30
NotifyAccess=all

[Install]
WantedBy=multi-user.target
EOF

  systemctl daemon-reload
  systemctl enable wave_rover.service
  ok "wave_rover.service created and enabled"
  info "Control: sudo systemctl start|stop|status wave_rover"
}

# =============================================================================
# Main
# =============================================================================
echo ""
echo "=========================================="
echo " Wave Rover RPi Safety Setup"
echo "=========================================="
echo ""

[[ "$DO_OVERLAY"  != "no" ]] && setup_overlay
[[ "$DO_WATCHDOG" != "no" ]] && setup_watchdog
[[ "$DO_SUDOERS"  != "no" ]] && setup_sudoers
setup_autostart

echo ""
echo "=========================================="
ok "Setup complete."
echo ""
warn "IMPORTANT: Reboot now for overlay FS to take effect:"
echo "  sudo reboot"
echo ""
echo "After reboot, to make config changes (packages, etc.):"
echo "  sudo overlayroot-chroot   (enter writable shell)"
echo "  OR: disable overlay in raspi-config, make changes, re-enable"
echo "=========================================="
