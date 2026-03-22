# Wave Rover ROS 2 — Python Workspace

**Platform:** Raspberry Pi 4 · LD19 LiDAR · PCA9685 Motor HAT · IMU (ESP32 HTTP or I2C)
**ROS 2 distro:** Jazzy (Ubuntu 24.04)

---

## Table of Contents

1. [Repository layout](#1-repository-layout)
2. [Package overview](#2-package-overview)
3. [How each node works](#3-how-each-node-works)
4. [Dynamic node manager](#4-dynamic-node-manager)
5. [Mobile web interface](#5-mobile-web-interface)
6. [Safe shutdown & filesystem protection](#6-safe-shutdown--filesystem-protection)
7. [Build instructions](#7-build-instructions)
8. [Running the system](#8-running-the-system)
9. [Topic & parameter reference](#9-topic--parameter-reference)
10. [Hardware wiring notes](#10-hardware-wiring-notes)

---

## 1. Repository layout

```
BPC-PRP/
├── cpp_project_template/      C++ waverower node (original, untouched)
├── waver-humble/              Gazebo simulation stack (untouched)
│   └── waver_description/     Robot URDF/xacro – used by BOTH stacks
├── wave_rover_ws/             ← THIS Python workspace
│   └── src/
│       ├── wave_rover_utils/
│       ├── wave_rover_control/
│       ├── wave_rover_sensors/
│       ├── wave_rover_navigation/
│       └── wave_rover_bringup/
└── ugv_base_general/          ESP32 Arduino firmware (untouched)
```

The two stacks (C++ and Python) are **independent**.
- Use `cpp_project_template` on the physical robot when you want the C++ keyboard-WASD control.
- Use `wave_rover_ws` for the structured Python system with web UI, dynamic node management, and Nav2.

---

## 2. Package overview

| Package | Role |
|---------|------|
| `wave_rover_utils` | Shared library – `PCA9685MotorDriver` and `MockMotorDriver` |
| `wave_rover_control` | All control nodes: motor controller, IMU stabilizer, mode manager, watchdog, node manager, shutdown |
| `wave_rover_sensors` | Sensor nodes: IMU, camera, web server |
| `wave_rover_navigation` | Nav2 + SLAM + EKF config files and launch files |
| `wave_rover_bringup` | Top-level launch files, robot params, web UI, mode switcher script |

---

## 3. How each node works

### `motor_controller_node`  (`wave_rover_control`)

Converts a `geometry_msgs/Twist` on `/cmd_vel` into PWM signals for the PCA9685 HAT.

**Math:**
```
half = wheel_base / 2                    # 0.10 m
v_l  = (linear_x - angular_z * half) / max_wheel_speed
v_r  = (linear_x + angular_z * half) / max_wheel_speed
pwm  = clamp(v * max_pwm * pwm_boost, -100, 100)
```

**Smoothing:** exponential moving average `smooth += alpha * (target - smooth)` prevents jerky motion.
**Watchdog:** if no `/cmd_vel` arrives for `cmd_vel_timeout_ms`, motors stop automatically.
**Mock mode:** set `use_mock: true` in params → runs without hardware, logs commands to terminal.

Key parameters (all runtime-adjustable via `ros2 param set`):

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `wheel_base` | 0.20 m | Track width center-to-center |
| `max_wheel_speed` | 0.35 m/s | Speed at 100 % PWM |
| `pwm_boost` | 2.35 | Overcomes static friction (matches C++ node) |
| `smooth_alpha` | 0.8 | 1.0 = instant, 0.1 = very smooth |
| `use_mock` | false | true = software-only, no I2C |

---

### `imu_stabilizer_node`  (`wave_rover_control`)

Corrects angular velocity error caused by wheel slip without encoders.

**How it works:**
```
error      = commanded_angular_z − measured_angular_z (from IMU)
correction = Kp * error  +  Ki * integral  +  Kd * derivative
output_angular_z = commanded_angular_z + correction
```

Published on `/cmd_vel_stable`. The motor controller subscribes to this instead of raw `/cmd_vel`.
Can be disabled with `enabled: false` in params — then passes through unchanged.

---

### `mode_manager_node`  (`wave_rover_control`)

Software mux that gates which source controls the robot.

```
/cmd_vel_teleop ─┐
                  ├─► mode_manager ──► /cmd_vel ──► motor_controller
/cmd_vel_nav    ─┘
```

| Mode | What happens |
|------|-------------|
| `manual` | Forwards `/cmd_vel_teleop`, blocks Nav2 |
| `auto` | Forwards `/cmd_vel_nav`, blocks keyboard |
| `toggle` | Flips between the two |

Publishes current mode (latched) on `/current_mode` — the web UI subscribes to this.

---

### `watchdog_node`  (`wave_rover_control`)

Safety node. Monitors `/cmd_vel`. If silent for `timeout_ms` (default 500 ms), publishes a zero-velocity Twist to stop the robot. Useful if the controlling node crashes or the network drops.

Sits inline:
```
/cmd_vel_raw ──► watchdog ──► /cmd_vel
```

---

### `node_manager_node`  (`wave_rover_control`)

**Dynamically starts and stops navigation stacks** based on the current task. Instead of running SLAM + Nav2 always (wastes RPi CPU/RAM), it launches only what is needed.

| Task (`/nav_mode`) | What gets started | What gets killed |
|--------------------|-------------------|-----------------|
| `slam` | `slam_only.launch.py` (SLAM Toolbox + EKF) | Previous task's process |
| `nav` | `nav_only.launch.py` (Nav2 + AMCL + map_server + EKF) | Previous task's process |
| `off` | Nothing | Previous task's process |

When switching:
1. Sends `SIGINT` to the old process group (clean ROS2 shutdown).
2. Waits up to `shutdown_timeout` seconds.
3. If it doesn't stop: `SIGKILL`.
4. Starts new launch file via `subprocess.Popen`.
5. Monitors for immediate crash (startup failure after 1.5 s).

Publishes current active task on `/nav_mode_status` at 1 Hz.

---

### `shutdown_node`  (`wave_rover_control`)

Listens for a shutdown request and calls `systemctl poweroff` (works without sudo on Ubuntu via polkit).

Trigger options:
1. **ROS topic:** `ros2 topic pub --once /shutdown std_msgs/msg/String "data: halt"`
2. **GPIO button:** wire a push-button between BCM pin 21 and GND, set `gpio_enabled: true` in params.
3. **Any other node** publishing to `/shutdown`.

---

### `imu_node`  (`wave_rover_sensors`)

Publishes `sensor_msgs/Imu` from two possible backends:

| Backend | How | When to use |
|---------|-----|-------------|
| `http` | Polls ESP32: `GET http://IP/js?json={"T":126}` | Default – ESP32 is the IMU host |
| `i2c` | Reads MPU6050/QMI8658 directly via smbus2 | If IMU wired directly to RPi |

Converts Euler angles (roll/pitch/yaw) → quaternion, fills covariance matrices.
Publishes to `/imu/data` and `/imu/raw` (alias).

---

### `camera_node`  (`wave_rover_sensors`)

Opens a USB or CSI camera via OpenCV, publishes:
- `/camera/image_raw` (sensor_msgs/Image, BGR8)
- `/camera/camera_info` (calibration info)
- `/camera/compressed` (JPEG, optional, saves bandwidth)

Reopens automatically if the camera disconnects.

---

### `web_server_node`  (`wave_rover_sensors`)

Serves the mobile web UI (`index.html`) at `http://ROBOT_IP:8888`.
Uses Python's built-in `HTTPServer` in a background thread — zero dependencies.

---

## 4. Dynamic node manager

The system is split into two layers:

**Always running** (started by `bringup.launch.py`):
```
robot_state_publisher   joint_state_publisher
imu_node                camera_node (optional)
imu_stabilizer_node     mode_manager_node
motor_controller_node   watchdog_node
node_manager_node       shutdown_node
web_server_node         rosbridge_websocket
```

**Started on demand** by `node_manager_node`:
```
nav_mode = slam  →  async_slam_toolbox_node + ekf_filter_node
nav_mode = nav   →  nav2 full stack + amcl + map_server + ekf_filter_node
nav_mode = off   →  nothing extra
```

**How to change task** (three equivalent ways):

```bash
# 1. Command line
ros2 topic pub --once /nav_mode std_msgs/msg/String "data: slam"

# 2. Interactive script
ros2 run wave_rover_bringup mode_switch   # option [1] or [2]

# 3. Mobile web UI – tap SLAM or NAV2 button
```

Switching takes ~2–10 seconds (SLAM/Nav2 need time to initialise).

---

## 5. Mobile web interface

Open `http://ROBOT_IP:8888` on any phone/tablet on the same WiFi.

### What it shows

| Section | Description |
|---------|-------------|
| **Connection** | Enter robot IP, connect button. IP saved to localStorage. |
| **Camera** | Live MJPEG stream from `/camera/image_raw` via `web_video_server`. |
| **Status** | Current drive mode, nav task, front distance (from `/lidar_sectors`), commanded linear/angular velocity, rosbridge latency. |
| **Drive mode** | Buttons: MANUAL / AUTONOMOUS. Buttons: SLAM (map) / NAV2 (goto) / OFF. |
| **Navigation goal** | Enter X, Y, Yaw in map frame → sends `NavigateToPose` action. STOP cancels. |
| **Joystick** | nippleJS virtual joystick → publishes to `/cmd_vel_teleop` at 10 Hz. Works with thumb in any direction. |
| **LiDAR canvas** | 260×260 px top-down scan visualisation from `/scan`. Robot = blue dot. Points = green dots. Grid rings every 1 m. |
| **E-STOP** | Publishes zero velocity + cancels Nav2 goal + switches to manual. |
| **Log** | Last 5 timestamped events. |

### Required packages on RPi

```bash
sudo apt install ros-jazzy-rosbridge-suite ros-jazzy-web-video-server
```

`rosbridge_websocket` (port 9090) and `web_server_node` (port 8888) start automatically with bringup.

---

## 6. Safe shutdown & filesystem protection

### Option A — Read-only overlayfs (best protection against power cuts)

Run once on the RPi:
```bash
sudo bash wave_rover_ws/src/wave_rover_bringup/scripts/setup_rpi_safe.sh
sudo reboot
```

After reboot: the SD card root is **read-only**. All writes go to a RAM tmpfs. If power is cut, nothing is half-written. Data written at runtime (logs, maps saved to `/tmp`) is lost on reboot — save maps to a USB drive or `/boot` partition before powering off.

To make changes to installed software:
```bash
sudo overlayroot-chroot   # enter a writable shell
# install packages, edit configs...
exit
# reboot to apply
```

### Option B — GPIO hardware shutdown button

Wire: `BCM pin 21 (header pin 40) → button → GND`

In `robot_params.yaml`:
```yaml
shutdown_node:
  ros__parameters:
    gpio_enabled: true
    gpio_pin: 21
```

Press the button → `shutdown_node` sends `systemctl poweroff` → clean shutdown → no corruption.

### Option C — Shutdown via ROS topic (software)

```bash
ros2 topic pub --once /shutdown std_msgs/msg/String "data: halt"
```

### Hardware watchdog

The setup script enables the BCM2835 hardware watchdog timer. If the Linux kernel stops petting it for >15 seconds (OS hangs, kernel panic), the RPi hardware-resets itself. Prevents a permanently hung robot.

### Systemd auto-start

The setup script installs `/etc/systemd/system/wave_rover.service`. The robot starts automatically on boot — no SSH needed after first setup.

```bash
sudo systemctl status wave_rover     # check status
sudo systemctl restart wave_rover    # restart
sudo systemctl stop wave_rover       # stop without shutdown
```

---

## 7. Build instructions

### First time

```bash
# Step 1 – install ROS2 dependencies
sudo apt install -y \
  ros-jazzy-robot-localization \
  ros-jazzy-slam-toolbox \
  ros-jazzy-nav2-bringup \
  ros-jazzy-rosbridge-suite \
  ros-jazzy-web-video-server \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-twist-mux \
  python3-smbus2 \
  python3-opencv

# Step 2 – build waver_description (provides the URDF)
source /opt/ros/jazzy/setup.bash
cd ~/Desktop/School/BPC-PRP/waver-humble
colcon build --packages-select waver_description
source install/setup.bash

# Step 3 – build the Python workspace
cd ~/Desktop/School/BPC-PRP/wave_rover_ws
colcon build
source install/setup.bash
```

### Subsequent builds (after code changes)

```bash
cd ~/Desktop/School/BPC-PRP/wave_rover_ws
colcon build --symlink-install   # faster, edits to Python files take effect immediately
source install/setup.bash
```

### Convenience aliases (add to ~/.bashrc)

```bash
alias sros='source /opt/ros/jazzy/setup.bash && \
            source ~/Desktop/School/BPC-PRP/waver-humble/install/setup.bash && \
            source ~/Desktop/School/BPC-PRP/wave_rover_ws/install/setup.bash'
alias bwav='cd ~/Desktop/School/BPC-PRP/wave_rover_ws && colcon build --symlink-install && sros'
```

---

## 8. Running the system

### Quick start — everything at once

```bash
sros   # source all workspaces

# Physical robot, development (mock motors, no hardware needed):
ros2 launch wave_rover_bringup full_system.launch.py use_mock:=true

# Physical robot, real hardware:
ros2 launch wave_rover_bringup full_system.launch.py

# Physical robot, start in navigation mode with a saved map:
ros2 launch wave_rover_bringup full_system.launch.py \
  nav_mode:=nav  map:=~/my_map.yaml
```

### Typical session — mapping then navigation

**Terminal 1 (robot):**
```bash
sros
ros2 launch wave_rover_bringup full_system.launch.py
# Robot starts in MANUAL mode, nav stack OFF
```

**Terminal 2 (robot or PC):**
```bash
sros
ros2 run wave_rover_bringup mode_switch
# Press [1] → starts SLAM, robot stays manual
```

**Terminal 3 (PC) — drive around:**
```bash
sros
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=cmd_vel_teleop
```

**OR** — use the virtual joystick in the web UI at `http://ROBOT_IP:8888`

**When map looks good:**
```bash
# In mode_switch menu:
[3] Save map  →  ~/my_map
[1] → [2]     →  switch to Auto + Nav2
[4] Send goal →  1.5 2.0    ← robot drives there, avoids obstacles
```

**OR** in RViz: click the **Nav2 Goal** button → click on the map.

### Launch arguments reference

| Argument | Default | Options |
|----------|---------|---------|
| `nav_mode` | `slam` | `slam` \| `nav` \| `off` |
| `map` | `""` | path to `.yaml` map file |
| `use_camera` | `false` | `true` \| `false` |
| `use_imu_stab` | `true` | `true` \| `false` |
| `use_rviz` | `true` | `true` \| `false` |
| `use_mock` | `false` | `true` = no motor HAT needed |
| `initial_mode` | `manual` | `manual` \| `auto` |
| `esp32_ip` | `192.168.0.224` | IP of ESP32 |
| `imu_backend` | `http` | `http` \| `i2c` |

### Runtime parameter changes

```bash
# Change max speed without restarting
ros2 param set /motor_controller_node max_wheel_speed 0.20

# Disable IMU stabilizer
ros2 param set /imu_stabilizer_node enabled false

# Adjust stabilizer gain
ros2 param set /imu_stabilizer_node Kp 0.5

# Change mode
ros2 topic pub --once /mode std_msgs/msg/String "data: auto"

# Start/stop nav tasks
ros2 topic pub --once /nav_mode std_msgs/msg/String "data: slam"
ros2 topic pub --once /nav_mode std_msgs/msg/String "data: nav"
ros2 topic pub --once /nav_mode std_msgs/msg/String "data: off"

# Safe shutdown
ros2 topic pub --once /shutdown std_msgs/msg/String "data: halt"
```

### Save a map

```bash
# While SLAM is running:
ros2 run nav2_map_server map_saver_cli -f ~/my_room
# Creates:  ~/my_room.pgm  (image)  +  ~/my_room.yaml  (metadata)
```

---

## 9. Topic & parameter reference

### Topics

| Topic | Type | Publisher | Description |
|-------|------|-----------|-------------|
| `/cmd_vel_teleop` | Twist | teleop / joystick | Keyboard or web joystick input |
| `/cmd_vel_nav` | Twist | Nav2 | Autonomous navigation commands |
| `/cmd_vel` | Twist | mode_manager → watchdog | Final command to motor controller |
| `/cmd_vel_stable` | Twist | imu_stabilizer | IMU-corrected commands |
| `/mode` | String | anywhere | `"manual"` \| `"auto"` \| `"toggle"` |
| `/current_mode` | String | mode_manager | Latched – current active mode |
| `/nav_mode` | String | anywhere | `"slam"` \| `"nav"` \| `"off"` |
| `/nav_mode_status` | String | node_manager | Current active nav task (1 Hz) |
| `/motor_pwm` | Float32MultiArray | motor_controller | Debug: `[left_pct, right_pct]` |
| `/imu/data` | Imu | imu_node | IMU measurements |
| `/scan` | LaserScan | LD19 driver | LiDAR scan |
| `/odom` | Odometry | ekf_filter_node | Estimated odometry (IMU only) |
| `/map` | OccupancyGrid | slam_toolbox | Live map |
| `/shutdown` | String | anywhere | Trigger safe poweroff |

### Key parameters (robot_params.yaml)

```yaml
motor_controller_node:
  wheel_base:         0.20    # track width [m]
  max_wheel_speed:    0.35    # [m/s]
  pwm_boost:          2.35    # stiction compensation
  smooth_alpha:       0.8     # motion smoothing
  use_mock:           false   # true = no hardware

imu_stabilizer_node:
  enabled: true
  Kp: 0.30
  Ki: 0.01
  Kd: 0.05

shutdown_node:
  gpio_enabled: false         # set true + connect button
  gpio_pin: 21                # BCM pin number
```

---

## 10. Hardware wiring notes

### Motor HAT (PCA9685 at I2C 0x40)

| PCA9685 channel | Signal | Motor |
|-----------------|--------|-------|
| 0 | PWMA | Left motor PWM |
| 1 | AIN1 | Left motor dir 1 |
| 2 | AIN2 | Left motor dir 2 |
| 3 | BIN1 | Right motor dir 1 |
| 4 | BIN2 | Right motor dir 2 |
| 5 | PWMB | Right motor PWM |

PWM frequency: **800 Hz** (set in `robot_params.yaml`).

### Shutdown button (optional)

```
RPi pin 40  (GPIO21 / BCM21)
      │
   [button]
      │
RPi pin 39  (GND)
```

Internal pull-up is enabled by the `gpiod` driver — no resistor needed.

### IMU (ESP32 HTTP backend, default)

The ESP32 runs `ugv_base_general` firmware. The `imu_node` polls:
```
GET http://192.168.0.224/js?json={"T":126}
→ { "r": roll, "p": pitch, "y": yaw,
    "ax": accel_x, "ay": accel_y, "az": accel_z,
    "gx": gyro_x,  "gy": gyro_y,  "gz": gyro_z }
```
Change the IP with `esp32_ip` parameter or launch argument.

### IMU (direct I2C, alternative)

Wire MPU6050 to RPi I2C:
```
MPU6050 VCC → 3.3V (pin 1)
MPU6050 GND → GND  (pin 6)
MPU6050 SDA → SDA  (pin 3, GPIO2)
MPU6050 SCL → SCL  (pin 5, GPIO3)
```
Set `imu_backend: i2c` in params or launch argument.

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| Motors don't move | No HAT / wrong I2C | Set `use_mock: true` to test; check `i2cdetect -y 1` |
| Robot drifts in auto mode | IMU noise / bad EKF covariance | Tune `ekf_params.yaml`, try disabling `imu_stabilizer` |
| Nav2 goal rejected | SLAM map not ready | Wait 10+ s after SLAM starts before sending goals |
| Web UI shows "Disconnected" | rosbridge not running | Check `ros2 node list | grep rosbridge` |
| Camera stream blank | `web_video_server` not installed | `sudo apt install ros-jazzy-web-video-server` |
| SLAM/Nav switch takes forever | Process not responding to SIGINT | Reduce `shutdown_timeout` or kill manually |
