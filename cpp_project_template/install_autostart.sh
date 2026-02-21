#!/bin/bash
# Inštalácia služby – lidar driver sa spustí pri štarte RPi a publikuje na /scan

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Použij aktuálneho užívateľa a jeho home (BP workspace = nadradený priečinok k cpp_project_template)
CURRENT_USER="${SUDO_USER:-$USER}"
CURRENT_HOME="${HOME:-/home/$CURRENT_USER}"
# Workspace s ldlidar_ros2 je o úroveň nad cpp_project_template
BP_INSTALL="$(cd "$SCRIPT_DIR/.." && pwd)/install"

echo "Používam: User=$CURRENT_USER, HOME=$CURRENT_HOME, BP install=$BP_INSTALL"
echo ""

# Dočasný súbor s nahradenými cestami
TMP_SERVICE=$(mktemp)
sed -e "s|User=aladix|User=$CURRENT_USER|g" \
    -e "s|Group=aladix|Group=$CURRENT_USER|g" \
    -e "s|/home/aladix|$CURRENT_HOME|g" \
    -e "s|BP_INSTALL_PLACEHOLDER|$BP_INSTALL|g" \
    systemd/ld19-lidar.service > "$TMP_SERVICE"

echo "Inštalácia automatického spustenia lidar drivera..."
sudo cp "$TMP_SERVICE" /etc/systemd/system/ld19-lidar.service
rm -f "$TMP_SERVICE"

sudo systemctl daemon-reload
sudo systemctl enable ld19-lidar.service
sudo systemctl start ld19-lidar.service

echo ""
echo "✅ Inštalácia dokončená!"
echo ""
echo "Status:"
sudo systemctl status ld19-lidar.service --no-pager | head -12

echo ""
echo "Lidar driver sa teraz spustí automaticky pri každom štarte (publikuje na /scan)."
echo ""
echo "Užitočné príkazy:"
echo "  sudo systemctl status ld19-lidar.service   - stav"
echo "  sudo systemctl restart ld19-lidar.service  - reštart"
echo "  sudo journalctl -u ld19-lidar.service -f    - logy naživo"
