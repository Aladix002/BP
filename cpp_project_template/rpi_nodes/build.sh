#!/bin/bash

set -e

# Source ROS 2 Jazzy
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
else
    echo "ERROR: ROS 2 Jazzy not found at /opt/ros/jazzy/"
    echo "Please install ROS 2 Jazzy first"
    exit 1
fi

# Prejdi do adresára projektu
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

# Vymaž starý build (voliteľné)
if [ "$1" == "clean" ]; then
    echo "Cleaning old build..."
    rm -rf build install log
fi

# Vytvor build adresár
mkdir -p build
cd build

# CMake konfigurácia s lokálnym install prefix
cmake .. -DCMAKE_INSTALL_PREFIX=../install

# Build
make -j$(nproc)

# Install
make install

# Vytvor setup.bash wrapper
cd ..
if [ ! -f install/setup.bash ]; then
    cat > install/setup.bash << 'EOF'
#!/bin/bash
# Wrapper script for ROS2 local_setup.bash
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
source "$SCRIPT_DIR/share/rpi_motor_node/local_setup.bash"
EOF
    chmod +x install/setup.bash
fi

echo ""
echo "Build completed!"
echo ""
echo "To run the ESP32 bridge node:"
echo "  source install/setup.bash"
echo "  ros2 run rpi_motor_node motor_esp32_bridge_node"
echo ""
echo "With custom serial port:"
echo "  ros2 run rpi_motor_node motor_esp32_bridge_node --ros-args -p serial_port:=/dev/ttyACM0"
echo ""

