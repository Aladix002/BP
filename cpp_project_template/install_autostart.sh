#!/bin/bash

echo "Inštalácia automatického spustenia lidar drivera..."

# Skopíruj service súbor
sudo cp systemd/ld19-lidar.service /etc/systemd/system/

# Obnov systemd
sudo systemctl daemon-reload

# Povol automatické spustenie
sudo systemctl enable ld19-lidar.service

# Spusti službu
sudo systemctl start ld19-lidar.service

echo ""
echo "✅ Inštalácia dokončená!"
echo ""
echo "Status:"
sudo systemctl status ld19-lidar.service --no-pager | head -10

echo ""
echo "Lidar driver sa teraz spustí automaticky pri každom štarte Raspberry Pi."
echo ""
echo "Užitočné príkazy:"
echo "  sudo systemctl status ld19-lidar.service  - Status"
echo "  sudo systemctl restart ld19-lidar.service  - Reštart"
echo "  sudo journalctl -u ld19-lidar.service -f  - Logy"
