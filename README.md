# Elicit prompty pre zdroje k bakalárke

Tento dokument je pripravený podľa celého textu v `bakalarka/projekt-01-kapitoly-chapters.tex`.
Cieľ: mať pripravené kategórie a konkrétne prompty pre [Elicit](https://elicit.com), aby sa dali rýchlo dohľadať vedecké zdroje a citácie k hardvéru, teórii aj implementácii.

## Ako prompty používať v Elicit

- Do Elicit zadávaj prompty po jednom (nie všetky naraz).
- Pri každej téme si vyber minimálne 3 až 5 relevantných papers.
- Uprednostňuj peer-reviewed články, survey papers a oficiálnu dokumentáciu.
- Sleduj rok vydania (najmä pri ROS 2, Nav2, micro-ROS a vision témach).
- Pri každej nájdenej práci si odlož: DOI/URL, krátky prínos, čo presne cituješ do textu.

## 1) Hardvér platformy a senzory

### 1.1 Raspberry Pi 5 v robotike

**Na čo citovať:** výkon, energetická efektivita, vhodnosť pre edge robotiku, porovnanie s predchádzajúcimi SBC.

**Elicit prompty:**
- "Raspberry Pi 5 performance for mobile robotics edge computing compared with Raspberry Pi 4"
- "Single-board computers in mobile robotics benchmark ARM Cortex-A76"
- "Power consumption and thermal behavior of Raspberry Pi 5 in robotics workloads"

### 1.2 Motor driver HAT / PCA9685 / H-mosty

**Na čo citovať:** princíp PWM riadenia DC motorov, H-bridge, limity presnosti bez enkodérov.

**Elicit prompty:**
- "PWM motor control with PCA9685 for differential drive robots"
- "H-bridge DC motor control theory and implementation in small mobile robots"
- "Open-loop differential drive control without wheel encoders limitations"

### 1.3 IMU MPU-6050 (akcelerometer, gyroskop, fúzia)

**Na čo citovať:** MEMS princíp akcelerometra/gyroskopu, šum, drift, komplementárny filter, porovnanie s Kalmanom.

**Elicit prompty:**
- "MEMS accelerometer and gyroscope principles in MPU-6050 type sensors"
- "Complementary filter vs Kalman filter for low-cost IMU orientation estimation"
- "Yaw drift characteristics of MPU-6050 in mobile robot applications"

### 1.4 LiDAR LD19 a 2D mapovanie

**Na čo citovať:** ToF princíp, presnosť 2D LiDARu, využitie pre obstacle avoidance a SLAM.

**Elicit prompty:**
- "2D LiDAR time-of-flight sensing principles for mobile robots"
- "Low-cost LiDAR performance evaluation for SLAM and obstacle avoidance"
- "LaserScan based navigation in differential drive robots"

### 1.5 Raspberry Pi Camera Module 3 (CSI, autofocus, libcamera)

**Na čo citovať:** vlastnosti Camera Module 3 pre robotiku (IMX708, autofocus), latencia/piepustnost CSI pipeline, používanie `libcamera`/`camera_ros` v ROS 2.

**Elicit prompty:**
- "Raspberry Pi Camera Module 3 IMX708 performance in robotics applications"
- "Autofocus behavior and latency of Raspberry Pi Camera Module 3 for computer vision"
- "libcamera based image pipeline performance on Raspberry Pi 5 with ROS 2"

## 2) Komunikačné rozhrania a integrácia

### 2.1 I2C, UART, USB serial v robotoch

**Na čo citovať:** spoľahlivosť zberníc, limity pri zdieľaní zariadení, latencia a robustnosť.

**Elicit prompty:**
- "I2C bus reliability in multi-device embedded robotic systems"
- "UART vs I2C vs USB serial communication trade-offs for robot sensors"
- "Practical failure modes of shared I2C in Linux-based robotics platforms"

### 2.2 Oddelenie IMU na Arduino Nano (anti-conflict architektúra)

**Na čo citovať:** architektonické oddelenie real-time snímania od hlavného SBC, zníženie konfliktov na zbernici.

**Elicit prompty:**
- "Offloading IMU acquisition to microcontroller in ROS-based robots"
- "Hybrid SBC plus microcontroller architectures for robust sensor integration"
- "Design patterns for separating motor control and IMU data acquisition"

### 2.3 ESP32 HTTP bridge vs micro-ROS

**Na čo citovať:** výhody/nevýhody HTTP bridge oproti natívnemu micro-ROS DDS prístupu.

**Elicit prompty:**
- "micro-ROS architecture and performance on ESP32 for ROS 2 integration"
- "Comparison of HTTP bridge and DDS-based communication in mobile robotics"
- "Latency and reliability evaluation of micro-ROS agent communication"

## 3) ROS 2 teória a systémová architektúra

### 3.1 ROS 2 vs ROS 1, DDS, QoS

**Na čo citovať:** decentralizácia, discovery, QoS profily, spoľahlivosť v nestabilnej sieti.

**Elicit prompty:**
- "ROS 2 DDS middleware advantages over ROS 1 in distributed robotics"
- "QoS tuning in ROS 2 for sensor streams and teleoperation"
- "ROS 2 communication reliability in lossy Wi-Fi environments"

### 3.2 Nodes/topics/services/actions a launch architektúra

**Na čo citovať:** best practices pre modulárny návrh uzlov a orchestrace launch súbormi.

**Elicit prompty:**
- "ROS 2 modular node architecture best practices for mobile robots"
- "Design patterns for ROS 2 launch files in multi-sensor robotic systems"
- "Service and action interfaces in ROS 2 for mode switching and autonomy"

### 3.3 TF, frame management, sensor fusion pipeline

**Na čo citovať:** dôležitosť konzistentných frame transformácií pre navigáciu.

**Elicit prompty:**
- "TF2 frame consistency issues in ROS 2 mobile robot navigation"
- "Common transformation errors in multi-sensor ROS navigation stacks"
- "Best practices for base_link odom map frame setup in Nav2"

## 4) Simulácia, modelovanie, SLAM a Nav2

### 4.1 URDF/Xacro a diferenciálny podvozok

**Na čo citovať:** kinematika differential drive, modelovanie robotov v URDF/Xacro.

**Elicit prompty:**
- "Differential drive kinematics for mobile robots theoretical foundations"
- "URDF and Xacro modeling practices for wheeled mobile robots"
- "Simulation fidelity considerations for differential drive robots"

### 4.2 Gazebo Sim + ROS bridge

**Na čo citovať:** simulácia senzorov, gap medzi simuláciou a realitou, bridge architektúra.

**Elicit prompty:**
- "Gazebo Sim integration with ROS 2 using ros_gz_bridge"
- "Sim-to-real gap in mobile robotics with LiDAR and camera simulation"
- "Validation of ROS 2 robot software in Gazebo before hardware deployment"

### 4.3 SLAM Toolbox a Nav2

**Na čo citovať:** 2D LiDAR SLAM, plánovanie trás, behavior trees v navigácii.

**Elicit prompty:**
- "2D LiDAR SLAM methods used in ROS 2 slam_toolbox"
- "Navigation2 architecture and performance for differential drive robots"
- "Behavior tree based navigation in ROS 2 Nav2 stack"

## 5) Riadenie pohybu a autonómne správanie

### 5.1 PID korekcia yaw z IMU

**Na čo citovať:** spätná väzba, tuning PID, anti-windup, deadband.

**Elicit prompty:**
- "PID yaw stabilization for differential drive mobile robots using IMU"
- "Anti-windup techniques for PID controllers in mobile robotics"
- "Practical tuning of low-cost IMU-based heading control"

### 5.2 Optický tok pre korekciu driftu

**Na čo citovať:** Lucas-Kanade vs Farneback, robustnosť pri rôznych podmienkach, limity bez odometrie.

**Elicit prompty:**
- "Lucas-Kanade optical flow for robot heading correction"
- "Farneback dense optical flow in real-time mobile robot control"
- "Vision-based drift correction for differential drive robots"

### 5.3 Reaktívne bludenie z LiDARu (sektorová logika)

**Na čo citovať:** obstacle avoidance heuristiky, reactive navigation, finite state machine.

**Elicit prompty:**
- "Reactive obstacle avoidance using 2D LiDAR sector partitioning"
- "Finite state machine navigation for autonomous wandering robots"
- "Comparison of reactive wandering and global planning in indoor robots"

### 5.4 Dynamické prepínanie režimov za behu

**Na čo citovať:** runtime reconfiguration, bezpečné prepínanie módov, concurrency v ROS 2 executore.

**Elicit prompty:**
- "Runtime mode switching in ROS 2 robots manual and autonomous control"
- "ROS 2 MultiThreadedExecutor and callback groups concurrency patterns"
- "Safe runtime reconfiguration of robot control nodes"

## 6) Počítačové videnie a sledovanie objektu

### 6.1 Klasická detekcia farby (HSV + blob)

**Na čo citovať:** prahovanie v HSV, morfológia, blob detekcia, limity vo variabilnom osvetlení.

**Elicit prompty:**
- "HSV color segmentation robustness in mobile robot vision"
- "Blob detection pipelines for real-time object tracking in ROS"
- "Lighting sensitivity of color-threshold based object detection"

### 6.2 Vizuálne servo riadenie (IBVS) a adaptívny PID

**Na čo citovať:** interakčná matica, väzba veľkosti objektu na odhad vzdialenosti, gain scheduling.

**Elicit prompty:**
- "Image-based visual servoing IBVS fundamentals for wheeled robots"
- "Gain scheduling PID for vision-based target tracking"
- "Visual tracking control using object size as distance proxy"

## 7) Teleprezencia, webové ovládanie a sieť

### 7.1 rosbridge + roslibjs + web teleop

**Na čo citovať:** webové HMI pre roboty, latencia WebSocket riadenia, komprimovaný stream obrazu.

**Elicit prompty:**
- "Web-based teleoperation of ROS robots using rosbridge and roslibjs"
- "Latency analysis of WebSocket teleoperation in mobile robotics"
- "Compressed image transport for low-bandwidth robot telepresence"

### 7.2 Human factors a bezpečnosť diaľkového ovládania

**Na čo citovať:** odozva rozhrania, operátorské chyby, failsafe pri výpadku spojenia.

**Elicit prompty:**
- "Human factors in mobile robot teleoperation interfaces"
- "Fail-safe mechanisms for networked robot teleoperation"
- "Usability metrics for touchscreen-based robot control"

## 8) Spoľahlivosť nasadenia a prevádzka

### 8.1 systemd, autorestart, watchdog prístupy

**Na čo citovať:** robustnosť dlhodobého behu robotického softvéru na edge zariadeniach.

**Elicit prompty:**
- "Systemd service supervision for autonomous robot software reliability"
- "Process restart strategies for ROS nodes in field robotics"
- "Operational robustness patterns for long-running mobile robot systems"

### 8.2 Integrita súborového systému pri náhlom výpadku

**Na čo citovať:** read-only rootfs, overlayfs, bezpečné vypínanie a odolnosť SD kariet.

**Elicit prompty:**
- "Read-only root filesystem strategies for Raspberry Pi robotics deployments"
- "OverlayFS for power-loss resilient embedded Linux systems"
- "SD card corruption mitigation in mobile robot platforms"

### 8.3 Kontajnery v ROS 2 (kedy áno / kedy nie)

**Na čo citovať:** trade-off medzi izoláciou a HW prístupom (I2C, USB, kamera).

**Elicit prompty:**
- "Containerized ROS 2 deployment on edge robots trade-offs"
- "Docker access to hardware peripherals in robotics I2C USB camera"
- "Native vs container ROS 2 performance on Raspberry Pi"

## 9) Prompt šablóny (na rýchle kopírovanie)

Použi tieto šablóny, keď chceš rýchlo vygenerovať nové dotazy:

- "Give me peer-reviewed papers (2018-2026) on <TOPIC> for mobile robotics, with focus on practical implementation and limitations."
- "Find survey and benchmark papers on <TOPIC>, especially methods used with ROS 2 and Raspberry Pi class hardware."
- "What are the best-cited methods for <TOPIC>, and which are suitable for low-cost differential-drive robots?"
- "Compare classical and modern approaches for <TOPIC>, including accuracy, latency, and computational cost."
- "Find papers that report real-world experiments (not only simulation) for <TOPIC> in indoor mobile robotics."

## 10) Odporúčaný výstup z Elicit (aby sa to dalo rovno vložiť do textu)

Pre každú nájdenú prácu si sprav krátky záznam:

- `Citation key návrh`: napr. `ibvs_gain_scheduling_2021`
- `Plná citácia`: autor, názov, rok, venue, DOI/URL
- `Prečo je relevantná`: 1 veta
- `Kde ju použiť v texte`: sekcia bakalárky (napr. IMU PID, optický tok, Nav2...)
- `Typ tvrdenia`: teória / implementácia / porovnanie / limitácia

Takto budeš mať konzistentný podklad na doplnenie bibliografie aj argumentácie v texte.
