# Wave Rover – architektúra ROS 2 systému

## Hardvérová konfigurácia

| Komponent | Popis |
|-----------|-------|
| RPi 4/5 | hlavný počítač |
| Waveshare Motor Driver HAT | 2× H-bridge, I2C `/dev/i2c-1`, addr `0x40` |
| 6-osové IMU | napr. MPU-6050, I2C `/dev/i2c-1`, addr `0x68` |
| LD19 LiDAR | USB `/dev/ttyUSB0`, 230400 Bd |
| Camera Module 3 | CSI / v4l2 |
| Motory | 4× DC bez enkodérov (kompenzácia cez IMU) |

---

## Executables a uzly

### `waverower` – motorový uzol (hlavný)

Spustenie: `ros2 run waverower waverower`

- Uzol: `wasd_motor_hat_node`
- Vstup (manuál): WASD/šípky z TTY + `Twist` na `/teleop_cmd_vel` z PC
- Vstup (auto): `Twist` na `/cmd_vel` (Nav2)
- Výstup: PWM na I2C Motor HAT
- Prepínač: parameter `control_mode` = `manual` | `auto` → meniteľný **za behu** (kláves **M** alebo `ros2 param set`)

```bash
ros2 param set /wasd_motor_hat_node control_mode auto
ros2 param set /wasd_motor_hat_node imu_correction true
ros2 param set /wasd_motor_hat_node imu_yaw_kp 0.15
```

Kľúčové parametre:

| Parameter | Default | Popis |
|-----------|---------|-------|
| `control_mode` | `manual` | `manual` / `auto` |
| `i2c_bus` | `1` | číslo I2C zbernice |
| `i2c_address` | `64` (=0x40) | adresa Motor HAT |
| `base_speed` | `1.0` | škálovanie rýchlosti 0.2–1.0 |
| `pwm_boost` | `2.35` | zosilnenie PWM |
| `snap_threshold` | `0.22` | pod touto hodnotou → plný plyn |
| `imu_correction` | `false` | IMU yaw korekcia pri priamej jazde |
| `imu_yaw_kp` | `0.15` | proporcionálny zisk korekcie |
| `imu_yaw_deadband` | `0.02` | ignoruje gyro pod touto hodnotou [rad/s] |
| `wheel_separation_m` | `0.20` | rozchod kolies [m] (pre auto) |
| `max_wheel_linear_m_s` | `0.35` | max rýchlosť kolesa [m/s] |

---

### `waverower_imu` – IMU uzol (MPU-6050, priamy I2C)

Spustenie: `ros2 run waverower waverower_imu`

- Uzol: `imu_i2c_node`
- Výstup: `sensor_msgs/Imu` na `/imu`

Parametre:

| Parameter | Default | Popis |
|-----------|---------|-------|
| `i2c_bus` | `1` | `/dev/i2c-X` |
| `i2c_address` | `0x68` | 0x68 alebo 0x69 (MPU-6050) |
| `publish_rate_hz` | `100.0` | Hz |
| `gyro_range_dps` | `250` | 250 / 500 / 1000 / 2000 °/s |
| `accel_range_g` | `2` | 2 / 4 / 8 / 16 g |
| `frame_id` | `imu_link` | TF frame |

> Iný čip? Zmeň register adresy v `src/nodes/imu_i2c.cpp` (blok `namespace {}` na začiatku súboru).

---

### `waverower_camera` – kamera

Spustenie: `ros2 run waverower waverower_camera`

- Vstup: `/camera/image_raw` (raw) alebo `/camera/compressed`
- Výstup: `/camera/compressed`, `/detected_objects`, `/detected_people`
- Voliteľná YOLO detekcia (parameter `yolo_model` = cesta k `.onnx`)

---

## Launch súbory

### 1. Manuálna jazda + SLAM (mapovanie)

```bash
ros2 launch waverower slam.launch.py
ros2 launch waverower slam.launch.py use_rviz:=true
# Uloženie mapy:
ros2 run nav2_map_server map_saver_cli -f ~/mapa_izba
```

### 2. Autonómna navigácia (Nav2 + SLAM)

```bash
ros2 launch waverower nav.launch.py
ros2 launch waverower nav.launch.py use_camera:=true imu_correction:=true use_rviz:=true
```

V RViz: použiť **"2D Nav Goal"** → kliknúť cieľovú pozíciu → robot naplánuje a prejde trasu.

### 3. Len motory (cmd_vel → HAT)

```bash
ros2 launch waverower cmd_vel_hat.launch.py control_mode:=auto
```

---

## Dynamická rekonf. software – demo ROS 2

Zmena správania **za behu bez reštartu**:

```bash
# Prepnutie manuál ↔ autonómny
ros2 param set /wasd_motor_hat_node control_mode manual
ros2 param set /wasd_motor_hat_node control_mode auto

# Zmena rýchlosti, korekcie, agresivity
ros2 param set /wasd_motor_hat_node base_speed 0.5
ros2 param set /wasd_motor_hat_node imu_correction true
ros2 param set /wasd_motor_hat_node snap_threshold 0.15

# Výpis / dump parametrov
ros2 param list /wasd_motor_hat_node
ros2 param dump /wasd_motor_hat_node

# Nav2 lifecycle uzly (zapínanie/vypínanie plánovača)
ros2 lifecycle set /planner_server deactivate
ros2 lifecycle set /planner_server activate
```

---

## Topicy (prehľad)

```
/scan                LaserScan      LD19 → slam_toolbox, Nav2
/imu                 Imu            waverower_imu → waverower (korekcia)
/cmd_vel             Twist          Nav2 → waverower (auto)
/teleop_cmd_vel      Twist          PC teleop → waverower (manual)
/camera/compressed   CompressedImage  stream na PC
/detected_objects    String         YOLO výsledky
/map                 OccupancyGrid  slam_toolbox → Nav2, RViz
```

---

## Kompenzácia driftu bez enkodérov (IMU)

1. **SLAM** (scan matching) – hlavný zdroj lokalizácie namiesto wheel odometrie.
2. **IMU yaw korekcia** – pri priamej jazde číta gyro Z a upravuje PWM:
   `l -= Kp·ω_z`, `r += Kp·ω_z`.
   Ladenie: začni s `imu_yaw_kp=0.10`, príliš vysoká hodnota = oscilácie.
3. **Pseudo-odom** – statická identita `odom → base_link`; SLAM robí `map → base_link`.

---

Pozri tiež: [NAV_AUTONOMY.md](NAV_AUTONOMY.md)
