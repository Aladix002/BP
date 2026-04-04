#!/usr/bin/env bash
# Spusti RViz na PC s Nav2 a SLAM mapou (robot beží na RPi).
# Použitie: ./pc_rviz.sh [RPi_IP]
# Príklad:  ./pc_rviz.sh 192.168.0.218

RPI_IP="${1:-192.168.0.218}"
RVIZ_CONFIG="/tmp/slam_nav.rviz"

source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0

echo "Kopírujem RViz konfig z RPi (${RPI_IP})..."
scp "aladix@${RPI_IP}:/home/aladix/Desktop/BP/waverower/params/slam.rviz" "${RVIZ_CONFIG}" || {
    echo "scp zlyhalo – skúšam bez kopírovania (použijem lokálny súbor ak existuje)"
    RVIZ_CONFIG="$(dirname "$0")/../params/slam.rviz"
}

echo "Spúšťam RViz2..."
rviz2 -d "${RVIZ_CONFIG}"
