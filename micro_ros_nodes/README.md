# Vzorové micro-ROS uzly pre motor a IMU

Tento adresár obsahuje **vzorový kód** pre micro-ROS na mikrokontroléri (napr. ESP32), ktorý môže spolupracovať s ROS 2 na Raspberry Pi (Humble alebo Jazzy). micro-ROS umožňuje, aby ESP32 vystupoval ako plnohodnotný účastník ROS 2 siete: publikuje témy (napr. IMU) a odoberá témy (napr. príkazy pre motory), namiesto vlastného HTTP protokolu.

## Čo je micro-ROS

**micro-ROS** [1] je projekt nadácie ROS 2, ktorý prináša ROS 2 (DDS, témy, služby, parametre) na embedded zariadenia s obmedzenou pamäťou a výkonom. Mikrokontrolér beží s **FreeRTOS** (na ESP32 typicky pod ESP-IDF), má na sebe micro-ROS knižnicu a cez transport (sériový port, WiFi alebo Ethernet) sa pripája k **micro-ROS agentovi** bežiacemu na Linuxe (RPi alebo PC). Agent je most medzi DDS a micro-ROS protokolom, takže uzly na RPi a uzly na ESP32 vidia navzájom svoje témy a služby v jednej ROS 2 doméne.

Kompatibilita s ROS 2: micro-ROS podporuje ROS 2 **Humble** a novšie (Iron, Jazzy). Na RPi sa používa rovnaký workspace ako pre bežné ROS 2 balíky; agent sa spúšťa ako `ros2 run micro_ros_agent micro_ros_agent ...`.

## Ako môžu fungovať spolu

- **Súčasná architektúra (bez micro-ROS):** Raspberry Pi beží uzly waverower (manual, lidar, bt\_auto, cmd\_mux, motor\_driver, camera, imu). Motor driver posiela príkazy na ESP32 cez **HTTP** (GET s JSON). IMU dáta prichádzajú z ESP32 tiež cez HTTP polling. ESP32 teda nie je v ROS 2 grafe.
- **Architektúra s micro-ROS:** ESP32 by bežal micro-ROS firmware, ktorý:
  - **odoberá** topic `/motor_cmd` (alebo `/auto_motor_cmd` / `/manual_motor_cmd` podľa návrhu) – typ `std_msgs/msg/Float64MultiArray` s `[left, right]` v rozsahu -1..1;
  - **publikuje** topic `/imu` – typ `sensor_msgs/msg/Imu`.
  Na RPi by už nebol potrebný motor_driver posielajúci HTTP ani IMU polling; cmd_mux a ostatné uzly by komunikovali priamo s ESP32 cez ROS 2 témy. Agent na RPi by zabezpečil prepojenie.

## Štruktúra vzoriek

- `motor_subscriber/` – ukážka micro-ROS **subscribera** na príkazy pre motory (Float64MultiArray).
- `imu_publisher/` – ukážka micro-ROS **publishera** IMU (sensor_msgs/msg/Imu).

Oba príklady sú koncipované ako jeden firmware (ESP-IDF + micro-ROS komponent), kde jeden node buď len odberá, alebo len publikuje; v reálnej aplikácii by jeden proces na ESP32 mohol mať oboje.

## Požiadavky na zostavenie

- ROS 2 Humble (alebo Jazzy) na PC alebo RPi
- Nástroj **micro_ros_setup** a vytvorenie firmware workspace pre platformu `esp32`
- ESP-IDF (odporúčaná verzia podľa dokumentácie micro-ROS pre ESP32)
- Pre flashovanie: USB pripojenie ESP32

Dokumentácia: [micro.ros.org](https://micro.ros.org/), návod na prvú aplikáciu: [First micro-ROS Application on FreeRTOS](https://micro.ros.org/docs/tutorials/core/first_application_rtos/freertos/). Pre ESP32: oficiálna dokumentácia micro-ROS a ESP-IDF komponent.

[1] micro-ROS – ROS 2 for microcontrollers, https://micro.ros.org/

## Placeholdery

- [ ] Presné verzie ESP-IDF a micro_ros_setup otestované s ROS 2 Humble/Jazzy.
- [ ] Konfigurácia agenta (sériový port vs WiFi) a parametre pre vašu sieť.
- [ ] Mapovanie topicov: či ESP32 publikuje priamo na `/motor_cmd` alebo na iný topic, ktorý ďalej spracuje uzol na RPi.
