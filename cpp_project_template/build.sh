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

# CMake konfigurácia
cmake ..

# Build
make -j$(nproc)

echo ""
echo "Build completed!"
echo ""
echo "To run the project:"
echo "  source install/setup.bash"
echo "  ros2 run prp_project prp_project"
echo ""

