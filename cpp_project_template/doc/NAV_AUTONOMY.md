# Autonómna navigácia (ROS 2 + Nav2) — plán a stav

Tento dokument dopĺňa [MOTOR_ARCH.md](MOTOR_ARCH.md) o mapovú navigáciu a Nav2.

## Cieľ

Robot sa má vedieť dostať na zvolenú polohu v mape (RViz **Navigate to Pose**, alebo vlastný **action klient** na `nav2_msgs/action/NavigateToPose`).

## Architektúra (vrstvy)

1. **Senzory:** `/scan` (LIDAR), voliteľne IMU, odometria z kolies → TF `odom` → `base_link`.
2. **Mapa:** `slam_toolbox` (tvorba) alebo `map_server` (uložená mapa).
3. **Lokalizácia:** `amcl` (mapa + scan + TF).
4. **Presnejšia odometria (voliteľné):** `robot_localization` EKF (IMU + kolieska + prípadne scan matching).
5. **Navigácia:** `nav2_bringup` — costmapy, plánovače, **`cmd_vel`** (`geometry_msgs/msg/Twist`).
6. **Tento balík — jeden uzol `wasd_motor_hat_node`:** parameter **`control_mode`** prepína **manuál (WASD + voliteľný Twist z PC)** vs **auto (`cmd_vel` z Nav2)**. I2C HAT vlastní vždy len jeden proces `waverower`.

```text
Nav2 → /cmd_vel → wasd_motor_hat_node (control_mode=auto) → PCA9685 / HAT
PC teleop → /teleop_cmd_vel → wasd_motor_hat_node (control_mode=manual) → HAT
```

## Manuál z PC (bez SSH klávesnice)

`waverower` musí bežať na **RPi** (I2C). Na **PC** nainštaluj teleop (napr. `sudo apt install ros-${ROS_DISTRO}-teleop-twist-keyboard`) a v **oboch** shelloch nastav rovnaký **`ROS_DOMAIN_ID`** a sieť tak, aby videli topic (`ros2 topic list` na PC ukáže uzly z Pi).

1. Na **RPi:** `control_mode=manual` (predvolené pri `ros2 run waverower waverower`).
2. Na **PC:**

```bash
export ROS_DOMAIN_ID=42   # rovnaké ako na Pi
source /opt/ros/jazzy/setup.bash   # alebo humble / iné, zhodné s Pi
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/teleop_cmd_vel
```

Uzol na Pi predvolene počúva **`/teleop_cmd_vel`** (parameter `manual_twist_topic`). `linear.x` → vpred/vzad, `angular.z` → otáčanie; škálovanie: `teleop_max_linear_m_s`, `teleop_max_angular_rad_s`.

V režime **auto** sa správy na `manual_twist_topic` **ignorujú** (neprekážajú Nav2 na `/cmd_vel`).

## Prepínanie manuál / auto (dynamická rekonfigurácia)

- Parameter **`control_mode`:** `manual` alebo `auto` (akceptuje aj `autonomous` → auto).
- Zmena za behu (bez reštartu uzla):

```bash
ros2 param set /wasd_motor_hat_node control_mode auto
ros2 param set /wasd_motor_hat_node control_mode manual
```

- Pri zmene režimu sa **vynuluje** pohybový stav (WASD decay, vyhladenie, posledný `cmd_vel`) a motory idú do bezpečného stavu cez rampu v ďalších tikoch.
- Z terminálu (ak beží `waverower` s TTY): kláves **M** prepína `manual` ↔ `auto` (interne cez `set_parameters`).

**Poznámka:** ROS 2 „vypína/zapína“ v tomto návrhu znamená **jeden motorový node** a logické vypnutie vetvy (v auto sa neberú klávesy do PWM; v manual sa ignoruje výstup z `cmd_vel` okrem príjmu správ). Samostatný druhý executable na HAT sa **nepoužíva** — dva procesy by bojovali o I2C.

## ROS 2 Actions

- Nav2 **bt_navigator** vystavuje **`NavigateToPose`**. Z RVizu alebo z C++/Python klienta pošleš cieľ; výsledok akcie = úspech / abort / timeout.
- Vlastná akcia nie je nutná, kým stačí štandard Nav2.

## Typický workflow

1. **Mapovanie (SLAM):** `ros2 run waverower waverower` (predvolene `control_mode=manual`) + `slam_toolbox` → uloženie mapy (`map_saver`).
2. **Prevádzka:** `map_server` + `amcl` + `/scan` + dobrá `odom`.
3. **Navigácia:** `nav2_bringup` + `waverower` s **`control_mode:=auto`** (napr. `ros2 launch waverower cmd_vel_hat.launch.py`).
4. **Cieľ:** RViz Nav2 Goal alebo vlastný action klient.

## Čo je už v repozitári

| Súčasť | Stav |
|--------|------|
| `waverower` | Jeden uzol: WASD + `cmd_vel` + HAT, prepínač `control_mode` |
| `launch/cmd_vel_hat.launch.py` | `waverower` s `control_mode=auto` a parametrami `cmd_vel` / I2C |
| Nav2 / slam / amcl bringup | Treba systémové balíky + vlastný launch podľa robota |
| TF `map`→`odom`→`base_link` | Treba nakonfigurovať (URDF / static TF / robot_state_publisher) |
| C++ `NavigateToPose` klient | Voliteľný ďalší krok |

## Parametre (auto časť + spoločné)

- **Režim:** `control_mode` — `manual` | `auto`.
- **I2C:** `i2c_bus`, `i2c_address`.
- **PWM / jazda:** `base_speed`, `pwm_boost`, `snap_threshold`, `turn_snap_threshold`, `smooth_alpha`, …
- **cmd_vel (auto):** `cmd_vel_topic`, `wheel_separation_m`, `max_wheel_linear_m_s`, `cmd_vel_timeout_ms`.
- **teleop (manual z PC):** `manual_twist_topic` (prázdny = neodoberať), `teleop_max_linear_m_s`, `teleop_max_angular_rad_s`, `teleop_invert_linear` (predvolene `true` — i / , z `teleop_twist_keyboard` sedí na smer motora).

## Ďalšie kroky (odporúčané poradie)

1. Overiť TF a frame_id u LIDARu (`base_link` / `laser`).
2. Spustiť SLAM + uložiť mapu.
3. Nastaviť `nav2_bringup` (footprint, max rýchlosti podľa `max_wheel_linear_m_s` a `cmd_vel` limitov).
4. Doladiť `wheel_separation_m` a škálovanie podľa reálnej jazdy.
5. (Voliteľne) Pridať C++ action klienta na sériu bodov.

## Spustenie s Nav2 (auto)

```bash
ros2 launch waverower cmd_vel_hat.launch.py
```

Alebo manuálne:

```bash
ros2 run waverower waverower --ros-args -p control_mode:=auto
```

Nav2 musí publikovať na rovnaký topic ako `cmd_vel_topic` (predvolene `/cmd_vel`).
