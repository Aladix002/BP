#!/bin/bash

# Distribuovaný ROS 2 systém
# RPI: LIDAR driver
# PC: Riadenie (prp_project)

RPI_IP="${1:-192.168.0.218}"
RPI_USER="${2:-rpi5}"
ROS_DOMAIN_ID="${3:-0}"

GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}=========================================="
echo "  Distribuovaný ROS 2 systém"
echo "==========================================${NC}"
echo "RPI: $RPI_USER@$RPI_IP (LIDAR)"
echo "PC: Riadenie"
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo ""

# Export ROS_DOMAIN_ID
export ROS_DOMAIN_ID=$ROS_DOMAIN_ID

# 1. Spusti LIDAR na RPI
echo -e "${GREEN}[1/2] Spúšťam LIDAR driver na RPI...${NC}"
ssh ${RPI_USER}@${RPI_IP} << EOF
    export ROS_DOMAIN_ID=$ROS_DOMAIN_ID
    cd ~/BPC-PRP/cpp_project_template
    source /opt/ros/jazzy/setup.bash 2>/dev/null || true
    source ~/lidar_ws/install/setup.bash 2>/dev/null || true
    
    # Zastav predchádzajúce
    pkill -f ldlidar || true
    
    # Spusti LIDAR
    ./scripts/start_d300_lidar.sh > /tmp/lidar.log 2>&1 &
    echo "LIDAR spustený (PID: \$!)"
EOF

sleep 2

# 2. Spusti riadenie na PC
echo -e "${GREEN}[2/2] Spúšťam riadenie na PC...${NC}"
cd /home/aladix/Desktop/School/BPC-PRP/cpp_project_template
source /opt/ros/jazzy/setup.bash 2>/dev/null || true
source install/setup.bash 2>/dev/null || true

# Zastav predchádzajúce
pkill -f prp_project || true

# Spusti projekt
ESP32_IP="${4:-192.168.0.224}"
ros2 run prp_project prp_project --ros-args -p esp32_ip:=$ESP32_IP
