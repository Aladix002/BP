#!/bin/bash

# Synchronizácia z RPI na PC (watch mode)
# Použitie: ./sync_from_rpi.sh [rpi_ip] [rpi_user]

RPI_IP="${1:-192.168.0.218}"
RPI_USER="${2:-rpi5}"
RPI_PATH="~/BPC-PRP"
PROJECT_DIR="/home/aladix/Desktop/School/BPC-PRP"

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m'

info() { echo -e "${BLUE}[INFO]${NC} $1"; }
success() { echo -e "${GREEN}[OK]${NC} $1"; }

info "Sledujem zmeny na RPI a synchronizujem na PC..."
info "Stlačte Ctrl+C pre ukončenie"
echo ""

# Prvá synchronizácia
info "Synchronizujem z RPI na PC..."
rsync -avz --progress \
  --exclude 'build' \
  --exclude 'install' \
  --exclude 'log' \
  --exclude '.git' \
  --exclude '*.o' \
  --exclude '*.a' \
  --exclude '*.so' \
  ${RPI_USER}@${RPI_IP}:${RPI_PATH}/ \
  "${PROJECT_DIR}/"

success "Počiatočná synchronizácia hotová"
echo ""

# Watch mode - sleduj zmeny na RPI
while true; do
    # Skontroluj zmeny na RPI (cez SSH)
    ssh ${RPI_USER}@${RPI_IP} "find ${RPI_PATH}/cpp_project_template -type f -newer /tmp/last_sync 2>/dev/null | head -1" > /dev/null 2>&1
    
    if [ $? -eq 0 ]; then
        echo ""
        info "Zistená zmena na RPI, synchronizujem..."
        rsync -avz --progress \
          --exclude 'build' \
          --exclude 'install' \
          --exclude 'log' \
          --exclude '.git' \
          --exclude '*.o' \
          --exclude '*.a' \
          --exclude '*.so' \
          ${RPI_USER}@${RPI_IP}:${RPI_PATH}/ \
          "${PROJECT_DIR}/"
        success "Synchronizácia hotová"
        ssh ${RPI_USER}@${RPI_IP} "touch /tmp/last_sync" 2>/dev/null
    fi
    
    sleep 2
done
