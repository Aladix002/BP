# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ROS 2 (Jazzy) autonomous robot project for a Wave Rover (Waveshare) platform. The upper computer (Raspberry Pi 4) runs ROS 2 decision-making; the lower computer (ESP32) runs the Waveshare `ugv_base_general` Arduino firmware for motor PID and sensors.

## Build & Run

**Main package** (`cpp_project_template/`):
```bash
source /opt/ros/jazzy/setup.bash
cd cpp_project_template
colcon build                          # from workspace root, or:
mkdir -p build && cd build && cmake .. && make -j$(nproc)

ros2 run waverower waverower          # manual mode
ros2 launch waverower cmd_vel_hat.launch.py   # autonomous / Nav2 mode
ros2 launch waverower rviz_lidar.launch.py    # RViz + LiDAR visualization
```

**Simulation stack** (`waver-humble/`):
```bash
cd waver-humble
./scripts/build.sh       # build Docker image
./scripts/run_docker.sh  # run container (CPU)
# inside container:
bros   # alias for colcon build
sros   # alias for source install/setup.bash
```

## Architecture

### Single-Process Design (cpp_project_template)

All ROS 2 nodes are compiled into one `waverower` executable to avoid I2C bus contention. Node implementations live in `src/nodes/` with matching headers in `include/nodes/`.

### Key Nodes

| Node | File | Role |
|------|------|------|
| `WasdMotorHatNode` | `wasd_motor_hat.*` | Central motor controller; owns I2C HAT (PCA9685); switches between Manual and Auto mode |
| `MotorHatI2c` | `motor_hat_i2c.*` | Low-level I2C driver for the PCA9685 PWM HAT |
| `BtAutoNode` | `behavioral_tree_auto.*` | Behavioral tree: Stop / Turn / Forward states based on LiDAR + camera |
| `CameraNode` | `camera.*` | Image capture + YOLO person detection |
| `LidarSectorsNode` | `lidar.*` | Extracts front sector from `LaserScan` |
| `ImuHttpNode` | `imu.*` | Polls IMU over HTTP, publishes `sensor_msgs/Imu` |
| `ManualTeleopNode` | `manual.*` | Keyboard teleop input |
| `CmdMuxNode` | `cmd_mux.*` | Multiplexes command sources |

### Dual Control Mode

`WasdMotorHatNode` has a `control_mode` parameter (`manual` | `auto`):
- **Manual** — reads WASD keyboard and optional Twist from a PC teleop node
- **Auto** — subscribes to `/cmd_vel` from Nav2; supports dynamic mode switching via ROS 2 parameter

Key motor parameters: `pwm_boost: 2.35`, `pwm_freq_hz: 800`, `wheel_separation_m: 0.20`, `max_wheel_linear_m_s: 0.35`.

### Navigation Stack (waver-humble)

Four ROS 2 packages provide the full simulation/navigation stack:
- `waver_description` — URDF/xacro robot model
- `waver_gazebo` — Gazebo simulation worlds and bridges
- `waver_nav` — Nav2 + SLAM Toolbox + AMCL + map server
- `waver_viz` — RViz configurations

### ESP32 Firmware (ugv_base_general)

Arduino/ESP32 project built with Arduino IDE. Handles closed-loop PID motor control, web UI, OLED display, and IMU reading. The `micro_ros_nodes/` directory contains experimental micro-ROS replacements for HTTP-based communication.

## Documentation

- `cpp_project_template/doc/NAV_AUTONOMY.md` — Nav2 integration, mode switching, NavigateToPose action
- `cpp_project_template/doc/MOTOR_ARCH.md` — Motor control and I2C HAT design rationale
- `cpp_project_template/systemd/README.md` — Systemd service setup for autostart
