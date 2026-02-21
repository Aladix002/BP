# BP - Autonomous Robot Project

ROS 2 (Jazzy) project for autonomous robot navigation.

## Project Structure

```
cpp_project_template/     # Main ROS 2 package
├── CMakeLists.txt       # Build configuration
├── package.xml          # ROS 2 package manifest
├── build.sh             # Build script
├── include/             # Header files
│   └── nodes/           # ROS node headers
└── src/                 # Source files
    ├── main.cpp         # Main entry point
    └── nodes/           # ROS node implementations
```

## Requirements

- ROS 2 Jazzy
- CMake 3.20+
- OpenCV
- SDL2

## Build

```bash
cd cpp_project_template
./build.sh
```

Or manually:
```bash
source /opt/ros/jazzy/setup.bash
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

## Run

```bash
source install/setup.bash
ros2 run prp_project prp_project
```

## Components

- **MotorController** - Motor control (left/right)
- **KinematicsOdometry** - Robot kinematics and odometry
- **LidarFilterNode** - LiDAR data processing
- **ImuNode** - IMU processing with calibration
- **CameraNode** - Image processing and ArUco detection
- **PidNode** - PID controller for navigation
- **ButtonListener** - Button detection
- **Line** - Line detection

## Remote Development

Ak vyvíjate na PC a spúšťate na Raspberry Pi (napr. kvôli USB LIDARu), pozrite si:

- **[REMOTE_DEVELOPMENT.md](REMOTE_DEVELOPMENT.md)** - Kompletný návod na vzdialený vývoj
- **Rýchly start:**
  ```bash
  # Automatická synchronizácia (watch mode)
  ./dev_remote.sh <rpi_ip> <rpi_user> watch
  
  # Alebo použite VS Code Remote SSH (odporúčané)
  # Pozri REMOTE_DEVELOPMENT.md
  ```

## Documentation

- [REMOTE_DEVELOPMENT.md](REMOTE_DEVELOPMENT.md) - Vzdialený vývoj PC → RPI
- [RPI_SETUP.md](RPI_SETUP.md) - Setup Raspberry Pi
- [cpp_project_template/D300_LIDAR_SETUP.md](cpp_project_template/D300_LIDAR_SETUP.md) - D300 LIDAR setup
- [cpp_project_template/ESP32_TOPICS.md](cpp_project_template/ESP32_TOPICS.md) - ESP32 komunikácia

## License

Apache-2.0

