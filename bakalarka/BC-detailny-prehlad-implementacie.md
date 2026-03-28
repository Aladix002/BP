# Detailný prehľad: BC text, práca „Mobilní kamera“, simulácia (Gazebo) a hardvérové zostavy

Tento dokument slúži ako **pracovný podklad** k tvojej bakalárskej práci v LaTeXu (`bakalarka/`) a k **implementácii v repozitári** (`gazebo/`, `waverower/`, …). Obsahuje mapovanie kapitol, porovnanie s príbuznou BC, rozbor dvoch hardvérových zostáv, kamery (libcamera), optical flow a návrhy na obrázky/diagramy.

**Aktualizácia:** Detailné **porovnanie s Onderkovou prácou** (*Mobilní kamera*), tabuľka rozdielov, odporúčania čo ešte doplniť a rozšírený **TODO zoznam** sú priamo v LaTeXu – na konci úvodnej kapitoly sekcia `\section*{Porovnanie s prácou Onderka...}` a na konci implementačnej časti sekcia `\section*{Zoznam úloh na doplnenie...}` v súbore `projekt-01-kapitoly-chapters.tex`.

---

## 1. Kde je „text BC“ a ako je členený

| Súbor | Úloha |
|--------|--------|
| `bakalarka/projekt.tex` | Hlavný dokument (titulná strana, nastavenia, vstupné body pre kapitoly). |
| `bakalarka/projekt-01-kapitoly-chapters.tex` | **Hlavný obsah práce** – kapitoly, sekcie, placeholdery. |
| `bakalarka/projekt-30-prilohy-appendices.tex` | Prílohy (SK). |
| `bakalarka/projekt-01-kapitoly-chapters-en.tex` | Anglická šablóna kapitol (u teba často len šablóna FIT). |
| `bakalarka/projekt-20-literatura-bibliography.bib` | Literatúra a citácie (`\cite{...}`). |

**Logická štruktúra obsahu** (podľa toho, čo už máš v `projekt-01-kapitoly-chapters.tex`):

1. **Úvod** – ciele, WAVE ROVER, ROS 2, odkazy na súvisiace práce.
2. **Hardware vybavenie** – RPi5, driver board, Wave Rover, IMU (teória), LiDAR, **kamera (nočné videnie + hardvér)**.
3. **Kamera na RPi5 + Ubuntu 24.04 + ROS 2 Jazzy** – **prečo nie legacy v4l2**, **libcamera**, **camera_ros**, témy, kalibrácia, RViz/rqt – *tu máš ešte `[PLACEHOLDER]` na väzbu na vlastnú implementáciu*.
4. **Komunikačné rozhrania** – I2C (a ďalšie podľa pokračovania súboru).
5. **ROS 2** – workspace, základné nástroje, RViz2, Gazebo (v texte je ešte starší popis integrácie).
6. **Integrácia senzorov** – všeobecné princípy + **súvis s prácami Skolek / Onderka**.
7. **micro-ROS** – teoretický/rámcový popis + placeholdery (ESP32).
8. **Implementácia** – architektúra `waverower` (uzly, HTTP→ESP32), **model a Gazebo**, návrh uzlov, SLAM/Nav2 ako placeholdery.

**Poznámka k konzistencii textu:** V kapitole o Gazebo je zmienka o `gazebo_ros` a „klasickom“ Gazebo; **tvoja aktuálna simulácia v `gazebo/` je postavená na Gazebo Sim (gz) + `ros_gz_*` bridge** – v BC je vhodné text zosúladiť s reálnym stackom (pozri sekciu 4).

---

## 2. Práca Onderka „Mobilní kamera realizovaná prostředky ROS2“ – čo je a čo z nej čerpáš

**Zdroj v bibliografii:** `projekt-20-literatura-bibliography.bib` → `\cite{mobilni_kamera}`

- **Autor:** Daniel Onderka  
- **Škola / typ:** VUT FIT Brno, **bakalárska práca**, **2024**  
- **Názov:** *Mobilní kamera realizovaná prostředky ROS2*  
- **V poznámke v .bib:** robot **Adeept AWR 4WD**, **RPi4**, **ROS 2 Iron**; teleprezencia; manuálne a autonómne ovládanie s vyhýbaním sa prekážkam; **Gazebo**, **slam_toolbox**, **Nav2**, **ros2_control**.

### 2.1 „Forma“ práce (ako ju typicky delíš v texte)

Onderkova práca je **klasická záverečná práca FIT**: úvod, súvisiaca práca, teória (ROS 2, robot, senzory), návrh systému, implementácia, merania, záver. Presné číslovanie kapitol **nemáš skopírované** v tomto repozitári (máš len citáciu a zhrnutie v úvode), ale **v tvojom úvode** ju používaš ako **referenčnú implementáciu**:

- **Distribuované uzly** (robot + PC)  
- **tf2**, YAML konfigurácie, remapovanie tém  
- **ros2_control**, **slam_toolbox**, **Nav2**  
- **Prepínanie manuál / autonómny** režim  

To je presne tá „forma“, ktorú v BC prirovnávaš k svojmu cieľu (Wave Rover, RPi5, LiDAR, kamera).

### 2.2 Druhá príbuzná práca – Skolek (`\cite{balancujici_robot}`)

- Balančný dvojkolesový robot, ROS 2 **Humble**, RPi4B, PID, IMU, enkodéry, Bluetooth, ultrazvuk.  
- Pre teba relevantné: **modularita balíkov**, **launch súbory**, **QoS**.

### 2.3 Porovnanie: Onderka / Skolek ↔ tvoja BC (obsahovo)

| Téma | Onderka (podľa .bib + úvod) | Tvoja práca (podľa kapitol + repo) |
|------|-----------------------------|-------------------------------------|
| Platforma | Adeept AWR 4WD, RPi4 | Wave Rover, **RPi5**, LiDAR LD19 |
| ROS 2 | Iron | **Jazzy** |
| Simulácia | Gazebo + Nav2 + SLAM | **Gazebo Sim + waver_sim / waver_nav**, Nav2, slam_toolbox |
| Teleprezencia / kamera | áno (v úvode) | **camera_ros / libcamera**, RViz Image, most gz→ROS |
| Riadenie pohybu | ros2_control (v .bib) | **diff-drive / skid v simulácii**; na hardvéri **Motor HAT (I2C)** alebo **ESP + HTTP** podľa zostavy |
| Embedded | nie je dôraz v citácii | **ESP32 + UGV Base General** *alebo* **micro-ROS** (máš teoretickú sekciu) |

---

## 3. Implementácia: simulácia „od človeka“ cez Gazebo po Nav2 a SLAM (tvoj `gazebo/` stack)

Nižšie je mapovanie na **skutočné súbory v repozitári** (nie len na LaTeX).

### 3.1 Spúšťanie

- Skript: `gazebo/run_sim.sh`  
  - čistí procesy, `RMW_FASTRTPS_USE_SHM=0`, `source /opt/ros/jazzy`, `source gazebo/install/setup.bash`  
  - voliteľný teleop terminál (`teleop_twist_keyboard` → `/cmd_vel_key`)  
  - bringup: **`waver_nav` full** ak existuje, inak **`waver_sim`** (`launch_sim.launch.py`).

### 3.2 Model robota (URDF / Xacro)

- V BC popisuješ časti v **Xacro** (`chassis`, kolesá, **camera.xacro**, LiDAR) – zodpovedá to konceptu **URDF + Gazebo pluginy**.  
- V projekte WaveR: balíky typu `waver_description` / `waver_sim` – **súbor `camera.xacro`** v tvojej práci je konceptuálne rovnaký ako v simulácii: **fixed joint** `camera_joint`, link `camera_link`, senzor v Gazebo s topicom (v gz často `/camera` → bridge na `/camera/image_raw`).

### 3.3 Gazebo Sim ↔ ROS 2

- Most: `waver_sim/config/ros_gz_bridge.yaml` – typicky **obraz** `/camera` → **`/camera/image_raw`**, `camera_info`, `/scan`, `/cmd_vel`, TF/clock podľa konfigurácie.  
- RViz: `waver_sim/config/main.rviz` – LaserScan, mapy, path, **Image** na `/camera/image_raw`.

### 3.4 Nav2 + SLAM

- Nav2: parametre v `waver_nav` (ak je v workspaci zbuildený), inak základ z upstream vzoru.  
- slam_toolbox: beží ako súčasť bringupu (podľa launch súborov).  
- **Manuálne riadenie**: teleop publikuje na `/cmd_vel` alebo remap (`/cmd_vel_key`); **Nav2** berie riadenie keď nedržíš klávesu (podľa nápisu v `run_sim.sh`).

### 3.5 Čo doplniť do BC textu oproti kódu

- Názov mosta: **`ros_gz_bridge`** / parameter bridge, nie len „gazebo_ros“.  
- Presné príkazy: `ros2 launch waver_sim launch_sim.launch.py` (a variant s `waver_nav`).  
- **Jeden RViz** – v texte zdôvodniť, prečo nepúšťaš duplicitný RViz z `waver_gazebo` (ak to tak máš v launchoch).

---

## 4. Dve hardvérové zostavy (ako si písal v požiadavke)

### 4.1 Zostava A: **ESP32 (UGV Base General) + HTTP**, bez Motor Driver HAT na RPi

**Čo to znamená v praxi**

- **Spodný počítač** na Waveshare riešení je **ESP32** na doske *General Driver for Robots* – firmware typu **`ugv_base_general`** (upstream: Waveshare / effectsmachine).  
- **Horný počítač** (RPi) posiela príkazy – v tvojej BC kapitole „Implementácia“ to zodpovedá uzlu, ktorý **HTTP** posiela JSON na ESP32; **IMU** môže byť **v ESP** (integrovaná na doske / firmware), nie externý MPU6050 na I2C RPi.  
- Komunikácia: **HTTP** (tvoj text) alebo v dokumentácii UGV aj **UART JSON** – pre BC si zvoľ jeden hlavný kanál a ten popíš konzistentne.

**Súvis s textom v `projekt-01-kapitoly-chapters.tex`**

- Popis **`motor_driver` + HTTP + `imu_http`** zodpovedá tejto vetve.  
- **Motor Driver HAT** sa tu **nepoužíva** – RPi neposiela PWM po I2C na HAT, ale **vyššiu vrstvu** na ESP.

### 4.2 Zostava B: **Motor Driver HAT (I2C) + MPU6050** na RPi (aktuálny `waverower`)

**Čo to znamená v praxi** (podľa `waverower/README.md`)

- Uzol **`wasd_motor_hat_node`**: odoberá **`/teleop_cmd_vel`** alebo **`/cmd_vel`**, podľa `control_mode`; výstup **PWM na Motor HAT** cez I2C (`0x40`).  
- **IMU:** samostatný balík **`mpu6050driver`** → topic **`/imu`** (`sensor_msgs/Imu`), voliteľná **yaw korekcia** pri jazde (`imu_correction`, `imu_yaw_kp`).  
- **LiDAR:** `ldlidar_ros2` → `/scan`.  
- **Kamera:** `camera_ros` / `ros2 run camera_ros camera_node` → `/camera/image_raw` (a komprimované témy); ďalšia vrstva môže byť `waverower_camera` (komprimácia, detekcie).

**Rozdiel voči zostave A**

- Žiadny HTTP motorový príkaz na ESP – **všetko na RPi** (okrem toho, čo riešiš sériovo na perifériách).  
- MPU6050 je **fyzicky na I2C RPi**, nie „vnorený“ v ESP firmware.

---

## 5. Kamera: libcamera, `camera_ros`, napojenie do ROS 2 a RViz

### 5.1 Prečo nie „klasický“ `v4l2_camera` na RPi5

V BC máš správny argument: **RPi5 + Ubuntu 24.04** používajú **libcamera** stack, nie starý V4L2-only model. Pre ROS 2 Jazzy sa používa balík **`camera_ros`** (autor Christian Rauch) – pozri `\cite{camera_ros}` v bibliografii.

### 5.2 Typický tok dát

1. **`camera_node`** z `camera_ros` publikuje napr. `/camera/image_raw`, `/camera/camera_info`.  
2. Pre sieť/teleprezenciu: **`/camera/image_raw/compressed`** alebo `image_transport` republish.  
3. **RViz**: display typu **Image**, topic **`/camera/image_raw`** (alebo compressed – podľa QoS a pluginu).  
4. **Optical flow** v `waverower`: default **`/camera/camera_node/image_raw/compressed`** – pozor na **presný názov topicu** (musí sedieť s tým, čo naozaj publikuješ; často treba zjednotiť remap v launch súbore).

### 5.3 Simulácia (Gazebo)

- Obraz neprichádza z CSI kamery, ale z **gz camera sensor** → **ros_gz_bridge** → `/camera/image_raw`.  
- Pre dokumentáciu: **jeden diagram** „Hardware vs Sim“ (pozri sekciu 8).

---

## 6. Optical flow – ako to máš implementované a na čo to je

### 6.1 Kde je kód

- Balík: `waverower`  
- Executables: `optical_flow` (sparse **Lucas–Kanade**), `optical_flow_dense` (**Farnebäck** – v CMakeLists).  
- Implementácia uzla: `waverower/src/nodes/optical_flow.cpp` (+ hlavička v `include/nodes/optical_flow.hpp`).

### 6.2 Lucas–Kanade vetva (zjednodušený popis)

1. **Vstup:** `sensor_msgs/CompressedImage` (JPEG) – dekódovanie cez OpenCV `imdecode` do **grayscale**.  
2. **Škálovanie:** max šírka **320 px** (výkon na RPi).  
3. **Features:** `goodFeaturesToTrack` na predošlom snímku.  
4. **Optical flow:** `calcOpticalFlowPyrLK` medzi `prev_gray_` a aktuálnym snímkom.  
5. Z **úspešne sledovaných bodov** sa spočíta priemer **horizontálneho posunu Δx** (normalizácia šírkou obrázka).  
6. Z toho sa vypočíta **`flow_correction_`** (obmedzené `max_correction`, zosilnené `correction_gain`).  
7. **Timer 20 Hz** číta posledný teleop `Twist` a **pripočíta korekciu k `angular.z`**, ale **len ak** robot ide dopredu (`|linear.x| > forward_threshold`) a používateľ **aktívne netočí** (`|angular.z| < steer_deadzone`).  
8. Výstup: **`/teleop_cmd_vel_corrected`** (default) – ďalší uzol (motor) by mal počúvať tento topic, ak chceš korekciu použiť.

### 6.3 Intuícia „vyrovnávanie trasy“

- Ak sa **obraz scény posúva doľava** vplyvom driftu robota doprava, stredová zložka optického toku to deteguje.  
- Korekcia **pridáva zákrutový príkaz** tak, aby sa robot **vrátil do smeru**, ktorý minimalizuje bočný posun vizuálnej scény – ide o **heuristiku**, nie plnú SLAM navigáciu.

### 6.4 Čo spomenúť v BC (odborne)

- Metódy: **Lucas–Kanade** (sparse), **Farnebäck** (dense) – OpenCV.  
- Obmedzenia: osvetlenie, textúry, motion blur, oneskorenie kompresie; prečo **iba pri jazde vpred** a bez ručného zásahu do riadenia.

---

## 7. Reálne zdroje (odporúčané na citácie / štúdium)

| Téma | Zdroj |
|------|--------|
| ROS 2 (prehľad) | Macenski et al., *Science Robotics* 2022 (`ros2` v .bib) – DOI v bibliografii |
| ROS 2 dokumentácia | https://docs.ros.org/en/jazzy/ |
| Nav2 | https://navigation.ros.org/ |
| slam_toolbox | https://github.com/SteveMacenski/slam_toolbox |
| Gazebo Sim | https://gazebosim.org/docs |
| ros_gz (bridge) | https://github.com/gazebosim/ros_gz |
| libcamera | https://libcamera.org/ |
| camera_ros (ROS 2 + libcamera) | https://github.com/christianrauch/camera_ros (`\cite{camera_ros}`) |
| OpenCV optical flow | https://docs.opencv.org/4.x/d4/dee/tutorial_optical_flow.html |
| Lucas–Kanade pôvodný článok | Lucas & Kanade (1981) – klasická citácia v texte o počítačovom videní |
| Waveshare Wave Rover | https://www.waveshare.com/wiki/WAVE_ROVER |
| UGV / ESP32 lower computer | https://github.com/effectsmachine/ugv_base_general (upstream README) |

---

## 8. TODO: obrázky, diagramy, grafy (kam ich dať v práci)

### 8.1 Architektúra a dáta

- [ ] **Diagram uzlov** (`rqt_graph` screenshot) – **simulácia**: `gz sim`, `ros_gz_bridge`, `robot_state_publisher`, `slam_toolbox`, `nav2`, `rviz2`, `teleop`.  
- [ ] **Diagram uzlov** – **hardvér Motor HAT**: `camera_ros`, `mpu6050driver`, `ldlidar_ros2`, `waverower`, voliteľne `optical_flow`.  
- [ ] **Diagram uzlov** – **hardvér ESP/HTTP**: RPi uzly → HTTP → ESP32 (oddelené „cloud“ vs robot).

### 8.2 TF a súradnice

- [ ] Strom **tf2** (`ros2 run tf2_tools view_frames`) – `base_link`, `laser`, `camera`, `map`, `odom`.  
- [ ] Schéma **prepočtu** `/cmd_vel` → koleso (diff / skid) – jedna rovnica + parametre z URDF.

### 8.3 Simulácia

- [ ] Screenshot **Gazebo** + ten istý moment v **RViz** (mapa + laser + kamera v Image paneli).  
- [ ] Obrázok **mosta** (tabuľka topicov gz ↔ ROS z `ros_gz_bridge.yaml`).

### 8.4 Kamera

- [ ] Ukážka **`rqt_image_view`** na `/camera/image_raw`.  
- [ ] (Voliteľné) histogram latencie alebo `ros2 topic hz /camera/image_raw`.

### 8.5 Optical flow

- [ ] Vizuálne: **predošlý vs aktuálny snímok** + nakreslené tracking body (OpenCV debug).  
- [ ] Graf: **časová os** `angular.z` bez / s korekciou; alebo stĺpcový graf priemernej odchýlky driftu.  
- [ ] Tabuľka parametrov: `correction_gain`, `max_correction`, `forward_threshold`, `steer_deadzone`, `min_features`.

### 8.6 Porovnanie zostáv

- [ ] Tabuľka **ESP zostava vs Motor HAT zostava** (komunikácia, IMU, kde beží PWM, kde je kamera).

---

## 9. Čo ešte „doplniť“ priamo do LaTeXu (placeholdery)

V `projekt-01-kapitoly-chapters.tex` máš výslovné `[PLACEHOLDER]` bloky – minimálne:

- Sekcia **kamera + vlastná implementácia** (väzba topicov, compressed vs raw).  
- **Gazebo** – finálny svet, launch príkaz, pluginy / bridge.  
- **SLAM** – či používaš len LiDAR alebo aj kameru (typicky **slam_toolbox + scan**).  
- **Schéma uzlov a systemd** – ak máš služby na RPi.

---

## 10. Stručné zhrnutie jednou vetou

- **Onderkova BC** je referenčná pre **ROS 2 + Gazebo + SLAM + Nav2 + teleprezenciu**.  
- **Tvoja BC** to prenáša na **Wave Rover + RPi5 + Jazzy**, s **dvoma reálnymi riadiacimi vetvami** (ESP/HTTP vs Motor HAT/I2C) a s **vlastnou úpravou jazdy** cez **optical flow** nad kamerou; **simulácia** je u teba v **`gazebo/`** cez **Gazebo Sim** a **ros_gz bridge**, nie cez starý `gazebo_ros` popis z generických učebníc.

---

*Vygenerované ako interný pracovný dokument k repozitáru BPC-PRP. Pri písaní záverečnej práce skontroluj ešte presné názvy launch súborov a topicov vo svojej aktuálnej vetve kódu.*
