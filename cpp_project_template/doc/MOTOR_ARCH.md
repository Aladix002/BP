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
- Vstup (manuál): `Twist` na `/teleop_cmd_vel` (napr. `teleop_twist_keyboard` s remapom)
- Vstup (auto): `Twist` na `/cmd_vel` (iný uzol / plánovač)
- Vstup z TTY (WASD) je v `main.cpp` vypnutý — ovládanie cez topicy.
- Výstup: PWM na I2C Motor HAT
- Prepínač: parameter `control_mode` = `manual` | `auto` → meniteľný **za behu** (`ros2 param set`)

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

### IMU – balík `mpu6050driver` (MPU-6050 cez I2C)

Vlastný IMU node v `waverower` bol odstránený. Použi komunitný driver (v workspace `src/ros2_mpu6050_driver`):

```bash
ros2 launch mpu6050driver mpu6050driver_launch.py
```

- Uzol: `mpu6050driver_node`
- Výstup: `sensor_msgs/Imu` na `/imu` (parametre v `mpu6050driver/share/mpu6050driver/params/mpu6050.yaml`)

---

### `waverower_camera` – kamera

Spustenie: `ros2 run waverower waverower_camera`

- Vstup: `/camera/image_raw` (raw) alebo `/camera/compressed`
- Výstup: `/camera/compressed`, `/detected_objects`, `/detected_people`
- Voliteľná YOLO detekcia (parameter `yolo_model` = cesta k `.onnx`)

---

## Launch súbory

### Jedným príkazom: motor + nový terminál s teleopom

Skript sám načíta `/opt/ros/<distro>/setup.bash` a `${WAVEROWER_WS}/install/setup.bash` — **nemusíš** nič `source`-ovať pred spustením.

**Odporúčané — z koreňa workspace** (`./` ako predtým):

```bash
cd ~/Desktop/BP
./waverower_teleop
```

Spustiteľný súbor `waverower_teleop` je v koreni `BP/` a nastaví `WAVEROWER_WS` podľa umiestnenia projektu.

**RPi cez SSH, teleop na PC:** na Raspberry Pi `./waverower_teleop` spustí len motor a vypíše príkazy pre `teleop_twist_keyboard` na osobnom počítači. Na oboch strojoch musí byť **rovnaký `ROS_DOMAIN_ID`** (ak ho nenastavíš, často ostane default `0`), **rovnaká sieť** a nesmú blokovať DDS (UDP/multicast). Na PC stačí nainštalovaný ROS 2 (nemusíš mať workspace s `waverower` — len teleop publikuje `Twist`).

**Len motor** (napr. lokálny desktop bez druhého okna): `./waverower_teleop --motor-only`

**Alternatívy** (izolovaný install):

```bash
bash ~/Desktop/BP/install/waverower/share/waverower/scripts/waverower_manual_teleop.sh
bash ~/Desktop/BP/cpp_project_template/scripts/waverower_manual_teleop.sh
```

Iný workspace: skopíruj `waverower_teleop` do jeho koreňa alebo `WAVEROWER_WS=/cesta/k/projekt bash .../waverower_manual_teleop.sh`.

**Cez `ros2 pkg prefix`** — najprv musí byť v prostredí workspace (inak „Package not found“):

```bash
source /opt/ros/jazzy/setup.bash
source ~/Desktop/BP/install/setup.bash
bash "$(ros2 pkg prefix waverower)/share/waverower/scripts/waverower_manual_teleop.sh"
```

---

### `manual_bringup.launch.py` – motory + voliteľne senzory

```bash
# Len motory (manuál, Twist na /teleop_cmd_vel)
ros2 launch waverower manual_bringup.launch.py

# + teleop z klávesnice na PC (cmd_vel → /teleop_cmd_vel)
ros2 launch waverower manual_bringup.launch.py use_teleop:=true

# + kamera (waverower_camera), IMU (mpu6050driver), LiDAR (ldlidar_ros2 LD19)
ros2 launch waverower manual_bringup.launch.py use_camera:=true use_imu:=true use_lidar:=true
```

IMU: musí byť zbuildený balík `mpu6050driver` v tom istom workspace (`colcon build --packages-select mpu6050driver`).

LiDAR: vyžaduje nainštalovaný balík `ldlidar_ros2` a správny `port_name` v jeho `ld19.launch.py` (predvolené `/dev/ttyUSB0`).

Ak sa ti `/imu` objaví hneď po boot-e bez spustenia launchu, skontroluj systemd: `systemctl list-unit-files | grep -iE 'imu|mpu|ros'` a prípadne `sudo systemctl disable --now <služba>`.

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

```

---

## Topicy (prehľad)

```
/scan                LaserScan      LD19 (ldlidar_ros2)
/imu                 Imu            mpu6050driver → waverower (voliteľná korekcia)
/cmd_vel             Twist          auto režim → waverower
/teleop_cmd_vel      Twist          manuálny teleop → waverower
/camera/compressed   CompressedImage  waverower_camera (voliteľne)
/detected_objects    String         YOLO výsledky (voliteľne)
```

---

## Kompenzácia driftu bez enkodérov (IMU)

1. **IMU yaw korekcia** – pri priamej jazde číta gyro Z a upravuje PWM:
   `l -= Kp·ω_z`, `r += Kp·ω_z`.
   Ladenie: začni s `imu_yaw_kp=0.10`, príliš vysoká hodnota = oscilácie.
2. Pre mapovanie / navigáciu môžeš neskôr doplniť externé balíky (napr. slam_toolbox, Nav2) a ponechať `waverower` v `control_mode:=auto` s `/cmd_vel`.
