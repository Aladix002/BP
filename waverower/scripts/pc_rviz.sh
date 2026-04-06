#!/usr/bin/env bash
# RViz na PC (Nav2 + SLAM), robot na RPi
# Pouzitie: ./pc_rviz.sh [RPi_IP]
# Priklad: ./pc_rviz.sh 192.168.0.218

set -euo pipefail
RPI_IP="${1:-raspberrypi.local}"

echo "Kopirujem RViz config z RPi (${RPI_IP})..."
if ! scp -q "pi@${RPI_IP}:~/BP/install/waverower/share/waverower/params/slam.rviz" /tmp/waverower_slam_pc.rviz 2>/dev/null; then
    echo "scp zlyhalo - skusam lokalny subor"
fi

echo "Spustam RViz2..."
source /opt/ros/jazzy/setup.bash
rviz2 -d /tmp/waverower_slam_pc.rviz 2>/dev/null || rviz2
