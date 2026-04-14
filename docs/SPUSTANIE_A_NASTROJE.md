# Spustenie, nástroje, parametre

`source install/setup.bash`
`source /opt/ros/jazzy/setup.bash`


---

## Spustenie

| Launch | Príkaz |
|--------|--------|
| Hlavný stack (motor, wander prepínač, LiDAR, IMU, kamera, web, SLAM, …) | `ros2 launch waverover runtime_stack.launch.py` |
| Motor + periférie, bez wander/web/SLAM v tomto súbore | `ros2 launch waverover manual_bringup.launch.py` |
| Len wander (auto + LiDAR) | `ros2 launch waverover wander.launch.py` |
| Web + rosbridge | `ros2 launch waverover web_teleop_ui.launch.py` |
| Lopta (action + BT) | `ros2 launch waverover ball_follow_standalone.launch.py` |
| Simulácia Gazebo + Nav2 + RViz | `ros2 launch waver_sim launch_sim.launch.py` |
| SLAM + jednoduchá navigácia **bez Nav2** (`simple_nav_node`, topic `/goal_pose`) | `ros2 launch waverover simple_nav.launch.py` |
| LiDAR LD19 | `ros2 launch ldlidar_ros2 ld19.launch.py` |

**`simple_nav` a RViz:** nie je to Nav2. Tlačidlá z panelu **Navigation 2** / **Nav2 Goal** posielajú action do `bt_navigator` — tu nebeží, robot nereaguje. Vyber nástroj **2D Goal Pose** (Set Goal) na hornom paneli — publikuje na `/goal_pose`. Over: `ros2 topic echo /goal_pose --once` po kliknutí. Ak nič: skontroluj `ROS_DOMAIN_ID` a Fixed Frame = `map`.

`slam.launch.py` → volá `manual_bringup` (ten **nespúšťa** slam). SLAM: `runtime_stack` s `use_slam:=true`.

**Prepínač režimu (runtime_stack):**

```bash
ros2 service call /waverover/switch_to_manual std_srvs/srv/Trigger
ros2 service call /waverover/switch_to_wander std_srvs/srv/Trigger
```

**Príklady:**

```bash
ros2 launch waverover runtime_stack.launch.py stack_mode:=wander
ros2 launch waverover runtime_stack.launch.py use_web:=false use_teleop:=true use_rviz:=true
ros2 launch waverover ball_follow_standalone.launch.py ball_color:=orange image_topic:=/camera/image_raw
ros2 launch waver_sim launch_sim.launch.py use_sim_time:=true
```

### `runtime_stack` — launch argumenty

| Arg | default | čo |
|-----|---------|-----|
| `stack_mode` | `manual` | `manual` / `wander` |
| `use_lidar` | `true` | LD19 |
| `use_imu` | `true` | |
| `use_camera` | `true` | treba `camera_ros` |
| `camera_id` | `0` | |
| `use_web` | `true` | HTTP 8080, ws 9090 |
| `use_teleop` | `false` | klávesnica → `/teleop_cmd_vel` |
| `use_slam` | `true` | `/scan` nutné |
| `use_ekf` | `false` | `params/ekf.yaml` |
| `use_rviz` | `false` | `params/slam.rviz` |
| `use_cmd_vel_odom` | `true` | `/odom` + TF |
| `use_robot_model` | `true` | URDF z `waver_sim` |
| `robot_model_file` | prázdne | vlastný xacro |
| `correction_mode` | `imu` | `imu` / `optical_flow` / `none` |
| `i2c_bus` / `i2c_address` | `1` / `64` | |
| `imu_serial_port` / `imu_baud_rate` / `imu_frame_id` | … / `115200` / `imu_link` | |
| `teleop_max_linear` / `teleop_max_angular` | `1.0` / `2.0` | |
| `wander_speed_scale` | `0.5` | |
| `forward_speed` / `turn_speed` | `auto` | číslo alebo `auto` |
| `threshold_m` | `0.30` | wander [m] |
| `lidar_rotation_deg` | `-90.0` | |
| `optical_flow_debug_show` | `false` | okno na robote |
| `optical_flow_debug_publish_image` | `false` | topic `/optical_flow/viz/compressed` |

`ROS_DOMAIN_ID` v launchi = `0` (rovnako na PC pri diaľkovom náhľade).

### `manual_bringup` — launch argumenty

| Arg | default |
|-----|---------|
| `control_mode` | `manual` / `auto` |
| `use_camera` | `false` |
| `camera_id` | `0` |
| `use_lidar` | `true` |
| `use_imu` | `true` |
| `use_teleop` | `false` |
| `use_cmd_vel_odom` | `true` |
| `use_robot_model` | `true` |
| `robot_model_file` | z `waver_sim` ak je |
| `correction_mode` | `imu` / `none` |
| `use_imu_kalman` | `false` |
| `imu_kalman_output` | `/imu/filtered` |
| `teleop_max_linear` / `teleop_max_angular` | `1.0` / `2.0` |
| `i2c_bus` / `i2c_address` | `1` / `64` |
| `imu_serial_port` / `imu_baud_rate` / `imu_frame_id` | … | |

### `wander` — launch argumenty

`i2c_bus`, `i2c_address`, `imu_serial_port`, `imu_baud_rate`, `threshold_m`, `forward_speed`, `turn_speed`, `lidar_rotation_deg`, `correction_mode` (`imu` / `none`).

```bash
ros2 param set /lidar_wander_node enabled false
```

### `web_teleop_ui`

| Arg | default |
|-----|---------|
| `serve_http` | `true` → `:8080` |

### `ball_follow_standalone`

| Arg | default |
|-----|---------|
| `image_topic` | `/camera/image_raw` |
| `cmd_topic` | `/cmd_vel` |
| `ball_color` | `orange` |

---

## Action `FollowBall` + BT

- Server: `/follow_ball` (`ball_follower_action.py`)
- Goal: `ball_color`, `max_duration_sec` (0 = bez limitu), `stop_when_found`, `fail_on_lost_sec`
- BT uzol `ball_follow_bt_runner`: parametre `ball_color`, `tick_rate_hz`, `find_timeout_sec`, `fail_on_lost_sec` — topic `/ball_follow_bt/active_behaviour`
- Follower YAML: `waverover/config/ball_follow.yaml`

```bash
ros2 action list -t
# send_goal: typ z výstupu vyššie
```

---

## Nástroje

| Čo | Príkaz |
|----|--------|
| RViz | `rviz2` alebo `rviz2 -d <súbor.rviz>` |
| SLAM config (robot) | `waverover/params/slam.rviz` |
| Sim | `waver_sim/config/main.rviz` |
| rqt | `rqt` |
| Obrázok | `rqt_image_view` |
| TF PDF | `ros2 run tf2_tools view_frames` |
| Topic | `ros2 topic list`, `echo`, `hz` |
| Param | `ros2 param list`, `get`, `set` |
| IMU séria (raw výstup) | `minicom -D /dev/ttyACM0 -b 115200` — port ako `imu_serial_port`; pred tým vypni launch / iný proces na tom istom zariadení |

---

## Parametre — kde meniť

1. **Launch:** `ros2 launch … arg:=hodnota` (mená v `.launch.py`)
2. **Beh:** `ros2 param set <node> <param> <hodnota>` (nižšie)
3. **Súbory:** `waverover/config/ball_follow.yaml`, `waverover/params/slam.yaml`, `waverover/params/ekf.yaml` → zmena + reštart uzla/launchu
4. **Topic remap:** `ros2 run … --ros-args -r starý:=nový`

LiDAR (`ldlidar_ros2`): parametre priamo v `ld19.launch.py` (port, baud, `frame_id`, …), nie cez launch args.

### Príkazy v termináli (za behu)

```bash
ros2 node list
ros2 param list /motor_hat_node
ros2 param describe /simple_nav_node drive_speed
ros2 param get /simple_nav_node drive_speed
ros2 param set /simple_nav_node drive_speed 0.5
```

Typy: reťazce v úvodzovkách, bool `true`/`false`, celé čísla bez desatinnej bodky.

### Často menené parametre podľa uzla

| Uzol | Príklad parametrov (runtime) |
|------|------------------------------|
| **`/motor_hat_node`** | `teleop_max_linear`, `teleop_max_angular`, `smooth_alpha`, `pwm_min` / `pwm_max`, `deadzone`, `imu_correction`, `imu_kp` / `imu_ki` / `imu_kd`, `wheel_base`, `cmd_vel_tank_mix`, `cmd_vel_invert_linear` |
| **`/lidar_wander_node`** | `enabled`, `threshold_m`, `forward_speed`, `turn_speed`, `lidar_rotation_deg`, `turn_timeout_s` |
| **`/simple_nav_node`** | `drive_speed`, `rotate_speed_max`, `rotate_done_deg`, `goal_tolerance_m`, `drive_steer_kp`, `drive_steer_max`, `approach_slowdown_m`, `rerotate_threshold_rad` |
| **`/cmd_vel_odom`** | `linear_scale`, `timeout_sec`, `update_rate_hz`, `use_imu_yaw` |
| **`/imu_serial_fusion_bridge`** | `baud_rate` (port väčšinou len reštart po zmene) |
| **`/optical_flow_node`** | `enabled`, `correction_gain`, `max_correction`, `debug_show`, `debug_publish_image` |
| **`/ball_follower`** | veľa (PID, rýchlosti, HSV) — `ros2 param list /ball_follower`; časť je v `config/ball_follow.yaml` |
| **`/ball_follow_bt_runner`** | `ball_color`, `tick_rate_hz`, `find_timeout_sec`, `fail_on_lost_sec` |
| **`/slam_toolbox`** | desiatky položiek — `ros2 param list /slam_toolbox`; väčšina ladenia v `params/slam.yaml` + reštart |

**Poznámka:** `control_mode` na motorovi mení aj `robot_mode_switch` služby; ručné `param set` môže byť v konflikte s webom, kým beží `runtime_stack`.
