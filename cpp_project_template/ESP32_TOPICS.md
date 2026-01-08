# ESP32 Bridge Topics

Tento dokument popisuje ROS 2 topics vytvorené pre komunikáciu s ESP32 cez HTTP.

## Node: `esp32_bridge`

Node, ktorý komunikuje s ESP32 cez HTTP a poskytuje ROS 2 topics pre motory a IMU.

### Parametre

- `esp32_ip` (string, default: "192.168.0.224") - IP adresa ESP32
- `imu_poll_rate_ms` (int, default: 50) - Frekvencia polling IMU dát v milisekundách (50ms = 20 Hz)

### Motor Control Topics

#### 1. `/bpc_prp_robot/cmd_motor_left` (std_msgs/msg/UInt8)
Ovládanie ľavej strany motorov.
- Hodnoty: **0-255**, kde **127 = stoj**
- 0-126 = dozadu (čím menšie, tým rýchlejšie dozadu)
- 128-255 = dopredu (čím väčšie, tým rýchlejšie dopredu)

**ESP32 príkaz:** Konvertuje sa na PWM `{"T":11,"L":<pwm>,"R":<pwm>}` kde PWM je -255 až +255

**Príklady:**
```bash
# Stoj (127)
ros2 topic pub --once /bpc_prp_robot/cmd_motor_left std_msgs/msg/UInt8 "{data: 127}"

# Dopredu (200 = stredná rýchlosť)
ros2 topic pub --once /bpc_prp_robot/cmd_motor_left std_msgs/msg/UInt8 "{data: 200}"

# Dozadu (50 = stredná rýchlosť)
ros2 topic pub --once /bpc_prp_robot/cmd_motor_left std_msgs/msg/UInt8 "{data: 50}"

# Maximálna rýchlosť dopredu
ros2 topic pub --once /bpc_prp_robot/cmd_motor_left std_msgs/msg/UInt8 "{data: 255}"
```

#### 2. `/bpc_prp_robot/cmd_motor_right` (std_msgs/msg/UInt8)
Ovládanie pravej strany motorov.
- Hodnoty: **0-255**, kde **127 = stoj**
- 0-126 = dozadu (čím menšie, tým rýchlejšie dozadu)
- 128-255 = dopredu (čím väčšie, tým rýchlejšie dopredu)

**ESP32 príkaz:** Konvertuje sa na PWM `{"T":11,"L":<pwm>,"R":<pwm>}` kde PWM je -255 až +255

**Príklady:**
```bash
# Stoj (127)
ros2 topic pub --once /bpc_prp_robot/cmd_motor_right std_msgs/msg/UInt8 "{data: 127}"

# Dopredu (200 = stredná rýchlosť)
ros2 topic pub --once /bpc_prp_robot/cmd_motor_right std_msgs/msg/UInt8 "{data: 200}"

# Dozadu (50 = stredná rýchlosť)
ros2 topic pub --once /bpc_prp_robot/cmd_motor_right std_msgs/msg/UInt8 "{data: 50}"

# Maximálna rýchlosť dopredu
ros2 topic pub --once /bpc_prp_robot/cmd_motor_right std_msgs/msg/UInt8 "{data: 255}"
```

#### 3. `/bpc_prp_robot/cmd_ros_ctrl` (geometry_msgs/msg/Twist)
ROS štandardný príkaz pre pohyb robota.
- `linear.x` - lineárna rýchlosť v m/s
- `angular.z` - uhlová rýchlosť v rad/s

**ESP32 príkaz:** `{"T":13,"X":<linear.x>,"Z":<angular.z>}`

**Príklad:**
```bash
ros2 topic pub /bpc_prp_robot/cmd_ros_ctrl geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.3}}"
```

#### 4. `/bpc_prp_robot/cmd_motor_speed` (std_msgs/msg/Float32MultiArray)
Priama kontrola rýchlosti kolies.
- `data[0]` - rýchlosť ľavého kolesa
- `data[1]` - rýchlosť pravého kolesa

**ESP32 príkaz:** `{"T":1,"L":<data[0]>,"R":<data[1]>}`

**Príklad:**
```bash
ros2 topic pub /bpc_prp_robot/cmd_motor_speed std_msgs/msg/Float32MultiArray "{data: [0.5, 0.5]}"
```

#### 5. `/bpc_prp_robot/cmd_motor_pwm` (std_msgs/msg/UInt8MultiArray)
Priama PWM kontrola motorov.
- `data[0]` - PWM ľavého motora (-255 až 255)
- `data[1]` - PWM pravého motora (-255 až 255)

**ESP32 príkaz:** `{"T":11,"L":<data[0]>,"R":<data[1]>}`

**Príklad:**
```bash
ros2 topic pub /bpc_prp_robot/cmd_motor_pwm std_msgs/msg/UInt8MultiArray "{data: [164, 164]}"
```

### Raw JSON Command Topic

#### `/bpc_prp_robot/cmd_json_raw` (std_msgs/msg/String)
**Všeobecný topic pre priame posielanie JSON príkazov na ESP32 z RPi.**

Tento topic prijíma JSON string a posiela ho priamo na ESP32 cez HTTP POST na `/js` endpoint.

**ESP32 príkaz:** Prijme akýkoľvek platný JSON príkaz podľa ESP32 dokumentácie

**Podporované príkazy (príklady):**
- `{"T":1,"L":0.5,"R":0.5}` - Speed control
- `{"T":11,"L":164,"R":164}` - PWM control
- `{"T":13,"X":0.1,"Z":0.3}` - ROS control
- `{"T":2,"P":200,"I":2500,"D":0,"L":255}` - Set motor PID
- `{"T":126}` - Get IMU data
- `{"T":130}` - Base feedback
- `{"T":131,"cmd":1}` - Enable base feedback flow
- `{"T":138,"L":1,"R":1}` - Set speed rate
- `{"T":405}` - Get WiFi info
- A všetky ostatné príkazy podľa ESP32 dokumentácie

**Príklady použitia:**
```bash
# Speed control
ros2 topic pub --once /bpc_prp_robot/cmd_json_raw std_msgs/msg/String "{data: '{\"T\":1,\"L\":0.5,\"R\":0.5}'}"

# PWM control
ros2 topic pub --once /bpc_prp_robot/cmd_json_raw std_msgs/msg/String "{data: '{\"T\":11,\"L\":100,\"R\":100}'}"

# ROS control
ros2 topic pub --once /bpc_prp_robot/cmd_json_raw std_msgs/msg/String "{data: '{\"T\":13,\"X\":0.1,\"Z\":0.3}'}"

# Set motor PID
ros2 topic pub --once /bpc_prp_robot/cmd_json_raw std_msgs/msg/String "{data: '{\"T\":2,\"P\":20,\"I\":2500,\"D\":0,\"L\":255}'}"

# Get WiFi info
ros2 topic pub --once /bpc_prp_robot/cmd_json_raw std_msgs/msg/String "{data: '{\"T\":405}'}"
```

**Poznámka:** JSON string musí byť správne escapovaný (použite jednoduché úvodzovky pre vonkajší string a dvojité úvodzovky pre JSON).

### IMU Topics

#### `/bpc_prp_robot/imu` (sensor_msgs/msg/Imu)
IMU dáta z ESP32 publikované automaticky.
- `linear_acceleration` - lineárne zrýchlenie (ax, ay, az) v m/s²
- `angular_velocity` - uhlová rýchlosť (gx, gy, gz) v rad/s
- `header.stamp` - časová značka
- `header.frame_id` - "imu_link"

**ESP32 príkaz:** `{"T":126}` (posiela sa automaticky podľa `imu_poll_rate_ms`)

**ESP32 odpoveď:** `{"T":1002,"ax":...,"ay":...,"az":...,"gx":...,"gy":...,"gz":...}`

**Príklad čítania:**
```bash
ros2 topic echo /bpc_prp_robot/imu
```

## Spustenie

### 1. Spustenie ESP32 Bridge node

```bash
ros2 run prp_project prp_project --ros-args -p esp32_ip:=192.168.0.224 -p imu_poll_rate_ms:=50
```

Alebo v launch súbore:
```xml
<node name="esp32_bridge" pkg="prp_project" exec="prp_project">
    <param name="esp32_ip" value="192.168.0.224"/>
    <param name="imu_poll_rate_ms" value="50"/>
</node>
```

### 2. Overenie, že ESP32 je dostupné

```bash
ping 192.168.0.224
curl http://192.168.0.224/
```

### 3. Testovanie topics

**Test motor control:**
```bash
# Ľavá a pravá strana (0-255, 127=stoj)
ros2 topic pub --once /bpc_prp_robot/cmd_motor_left std_msgs/msg/UInt8 "{data: 200}"
ros2 topic pub --once /bpc_prp_robot/cmd_motor_right std_msgs/msg/UInt8 "{data: 200}"

# ROS ctrl
ros2 topic pub --once /bpc_prp_robot/cmd_ros_ctrl geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}"

# Motor speed
ros2 topic pub --once /bpc_prp_robot/cmd_motor_speed std_msgs/msg/Float32MultiArray "{data: [0.3, 0.3]}"

# Motor PWM
ros2 topic pub --once /bpc_prp_robot/cmd_motor_pwm std_msgs/msg/UInt8MultiArray "{data: [100, 100]}"
```

**Test IMU:**
```bash
ros2 topic echo /bpc_prp_robot/imu
```

## Poznámky

- ESP32 musí byť pripojené k WiFi sieti a dostupné na zadané IP adrese
- IMU dáta sa získavajú periodicky podľa `imu_poll_rate_ms`
- Všetky príkazy sa posielajú cez HTTP POST na `http://<esp32_ip>/js`
- ESP32 odpovedá JSON odpoveďou, ktorá sa parsuje a publikuje ako ROS topics

## Troubleshooting

1. **ESP32 nie je dostupné:**
   - Skontrolujte IP adresu: `ros2 param get /esp32_bridge esp32_ip`
   - Skontrolujte WiFi pripojenie ESP32
   - Použite `find_esp32.sh` skript na vyhľadanie ESP32

2. **IMU dáta neprichádzajú:**
   - Skontrolujte, či ESP32 podporuje IMU príkaz `{"T":126}`
   - Skontrolujte `imu_poll_rate_ms` parameter
   - Pozrite sa na logy: `ros2 topic echo /rosout`

3. **Motor príkazy nefungujú:**
   - Skontrolujte, či ESP32 prijíma HTTP príkazy
   - Testujte priamo cez curl: `curl -X POST http://192.168.0.224/js -d '{"T":13,"X":0.1,"Z":0.0}'`

