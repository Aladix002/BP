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

## License

Apache-2.0

