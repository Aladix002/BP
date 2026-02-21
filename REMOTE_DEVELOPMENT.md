# Vzdialený vývoj - PC → Raspberry Pi

Tento dokument popisuje, ako vyvíjať kód na PC a spúšťať ho na Raspberry Pi, kde je pripojený LIDAR cez USB.

## Prehľad riešení

### 1. **VS Code Remote SSH** (Odporúčané) ⭐
Najlepšie riešenie pre vývoj. Umožňuje editovať súbory priamo na RPI cez SSH.

### 2. **Automatická synchronizácia (watch mode)**
Sleduje zmeny v súboroch a automaticky ich synchronizuje na RPI.

### 3. **SSHFS (Filesystem mounting)**
Pripojí filesystem RPI ako lokálny adresár na PC.

### 4. **Manuálna synchronizácia**
Použitie `sync_to_rpi.sh` alebo `dev_remote.sh` skriptov.

---

## Riešenie 1: VS Code / Cursor Remote SSH (Najlepšie)

### Inštalácia

**Pre VS Code alebo Cursor:**

1. **Nainštalujte Remote SSH extension:**
   - Otvorte **VS Code** alebo **Cursor**
   - Stlačte `Ctrl+Shift+X` (Extensions / Extensions Marketplace)
   - Vyhľadajte "**Remote - SSH**" (od Microsoft)
   - Nainštalujte extension
   
   **Poznámka:** Cursor je založený na VS Code, takže podporuje všetky VS Code extensions vrátane Remote SSH!

2. **Nastavenie SSH konfigurácie:**

   Vytvorte/upravte `~/.ssh/config` na PC:
   ```bash
   Host rpi
       HostName 192.168.1.100  # IP adresa vašej RPI
       User pi                  # Používateľ na RPI
       IdentityFile ~/.ssh/id_rsa  # Voliteľné: SSH kľúč
   ```

3. **Pripojenie:**
   - Stlačte `F1` alebo `Ctrl+Shift+P` (Command Palette)
   - Zadajte "**Remote-SSH: Connect to Host...**"
   - Vyberte "rpi" (alebo zadajte `pi@192.168.1.100`)
   - VS Code / Cursor sa pripojí na RPI
   - Môže sa zobraziť okno s výberom platformy (Linux) - vyberte správnu

4. **Otvorenie projektu:**
   - Po pripojení otvorte adresár: `~/BPC-PRP`
   - Alebo: `File` → `Open Folder...` → `/home/pi/BPC-PRP`
   - Teraz môžete editovať súbory priamo na RPI!

### Výhody
- ✅ Editovanie súborov priamo na RPI
- ✅ IntelliSense a debugger fungujú
- ✅ Integrovaný terminál na RPI
- ✅ Git funguje priamo na RPI
- ✅ Žiadna synchronizácia potrebná

### Práca s projektom

```bash
# V integrovanom termináli VS Code / Cursor (už na RPI):
# Terminál sa automaticky pripojí na RPI po pripojení cez Remote SSH
cd ~/BPC-PRP/cpp_project_template
colcon build
source install/setup.bash
ros2 run prp_project prp_project --ros-args -p esp32_ip:=192.168.0.224
```

### Tipy pre Cursor

- **AI funkcie fungujú aj na vzdialených súboroch!** Cursor AI môže navrhovať zmeny priamo v súboroch na RPI
- **IntelliSense** funguje normálne na vzdialených súboroch
- **Git** funguje priamo na RPI - môžete commitovať zmeny bez synchronizácie

---

## Riešenie 2: Automatická synchronizácia (Watch Mode)

### Použitie `dev_remote.sh` skriptu

```bash
# Urobte skript spustiteľný
chmod +x dev_remote.sh

# Watch mode - automaticky synchronizuje zmeny
./dev_remote.sh 192.168.1.100 pi watch

# Alebo s vlastnou IP
./dev_remote.sh <rpi_ip> <rpi_user> watch
```

### Iné príkazy

```bash
# Len synchronizácia
./dev_remote.sh 192.168.1.100 pi sync

# Synchronizácia + Build
./dev_remote.sh 192.168.1.100 pi build

# Spustenie na RPI
./dev_remote.sh 192.168.1.100 pi run 192.168.0.224

# Zastavenie na RPI
./dev_remote.sh 192.168.1.100 pi stop

# Sync + Build + Run (všetko naraz)
./dev_remote.sh 192.168.1.100 pi sbr 192.168.0.224
```

### Workflow

1. **Spustite watch mode:**
   ```bash
   ./dev_remote.sh 192.168.1.100 pi watch
   ```

2. **Editujte súbory na PC** - automaticky sa synchronizujú

3. **Na RPI (cez SSH v inom termináli):**
   ```bash
   ssh pi@192.168.1.100
   cd ~/BPC-PRP/cpp_project_template
   colcon build
   source install/setup.bash
   ros2 run prp_project prp_project --ros-args -p esp32_ip:=192.168.0.224
   ```

---

## Riešenie 3: SSHFS (Filesystem Mounting)

### Inštalácia

```bash
# Na PC
sudo apt-get install sshfs
```

### Pripojenie RPI filesystemu

```bash
# Vytvorte mount point
mkdir -p ~/rpi_mount

# Pripojte RPI filesystem
sshfs pi@192.168.1.100:~/BPC-PRP ~/rpi_mount

# Teraz môžete editovať súbory v ~/rpi_mount
cd ~/rpi_mount/cpp_project_template
```

### Odpojenie

```bash
fusermount -u ~/rpi_mount
```

### Výhody
- ✅ Editovanie súborov ako lokálne
- ✅ Automatická synchronizácia

### Nevýhody
- ⚠️ Môže byť pomalšie pri veľkých súboroch
- ⚠️ Vyžaduje stabilné pripojenie

---

## Riešenie 4: Manuálna synchronizácia

### Použitie `sync_to_rpi.sh`

```bash
./sync_to_rpi.sh 192.168.1.100 pi
```

### Potom na RPI

```bash
ssh pi@192.168.1.100
cd ~/BPC-PRP/cpp_project_template
colcon build
source install/setup.bash
ros2 run prp_project prp_project --ros-args -p esp32_ip:=192.168.0.224
```

---

## Odporúčaný workflow

### Pre každodenný vývoj:

1. **Použite VS Code Remote SSH** (Riešenie 1)
   - Najrýchlejšie a najpohodlnejšie
   - Žiadna synchronizácia potrebná

2. **Alebo kombinácia:**
   - Editovanie na PC (všetky nástroje)
   - `dev_remote.sh watch` pre automatickú synchronizáciu
   - SSH na RPI pre build a run

### Pre testovanie:

```bash
# Na PC - synchronizuj a spusti
./dev_remote.sh 192.168.1.100 pi sbr 192.168.0.224

# Alebo krok po kroku:
./dev_remote.sh 192.168.1.100 pi sync
./dev_remote.sh 192.168.1.100 pi build
./dev_remote.sh 192.168.1.100 pi run 192.168.0.224
```

---

## SSH bez hesla (Odporúčané)

Pre pohodlnejšiu prácu nastavte SSH kľúče:

```bash
# Na PC - vygenerujte SSH kľúč (ak nemáte)
ssh-keygen -t rsa -b 4096

# Skopírujte kľúč na RPI
ssh-copy-id pi@192.168.1.100

# Teraz sa pripojíte bez hesla
ssh pi@192.168.1.100
```

---

## Spustenie LIDAR drivera na RPI

LIDAR musí bežať na RPI (je pripojený cez USB):

```bash
# Na RPI (cez SSH alebo VS Code Remote SSH)
cd ~/BPC-PRP/cpp_project_template
./scripts/start_d300_lidar.sh
```

Alebo manuálne:

```bash
# Na RPI
source /opt/ros/jazzy/setup.bash
source ~/lidar_ws/install/setup.bash
ros2 launch ldlidar_stl_ros2 ldlidar.launch.py \
    serial_port:=/dev/ttyUSB0 \
    serial_baudrate:=230400 \
    frame_id:=lidar_link \
    lidar_type:=LD19
```

---

## Monitorovanie z PC

Aj keď kód beží na RPI, môžete monitorovať ROS 2 topicy z PC:

```bash
# Na PC - nastavte ROS_DOMAIN_ID (ak používate)
export ROS_DOMAIN_ID=0

# Alebo nastavte ROS_MASTER_URI (ak používate ROS 1)
# export ROS_MASTER_URI=http://192.168.1.100:11311

# Zobrazte topicy
ros2 topic list

# Monitorujte LIDAR dáta
ros2 topic echo /bpc_prp_robot/lidar
```

**Poznámka:** Pre ROS 2 cez sieť potrebujete:
- Rovnaký `ROS_DOMAIN_ID` na PC aj RPI
- Alebo použiť ROS 2 DDS discovery (multicast)

---

## Riešenie problémov

### "Permission denied" pri SSH
```bash
# Skontrolujte SSH kľúče
ssh-add ~/.ssh/id_rsa
```

### "Connection refused"
```bash
# Skontrolujte, či SSH beží na RPI
ssh pi@192.168.1.100 "sudo systemctl status ssh"
```

### VS Code / Cursor Remote SSH sa nepripája
- Skontrolujte IP adresu RPI: `ping 192.168.1.100`
- Skontrolujte SSH konfiguráciu: `ssh -v pi@192.168.1.100`
- V Cursore: Skontrolujte, či je extension nainštalovaný (`Ctrl+Shift+X` → vyhľadajte "Remote - SSH")
- Ak sa nepripojí, skúste manuálne: `ssh pi@192.168.1.100` v termináli

### LIDAR sa nenašiel na RPI
```bash
# Na RPI
lsusb
ls -la /dev/ttyUSB*
./scripts/check_lidar.sh
```

---

## Zhrnutie

| Riešenie | Rýchlosť | Pohodlie | Odporúčanie |
|----------|----------|----------|-------------|
| VS Code Remote SSH | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ | ✅ Najlepšie |
| Watch mode | ⭐⭐⭐⭐ | ⭐⭐⭐⭐ | ✅ Veľmi dobré |
| SSHFS | ⭐⭐⭐ | ⭐⭐⭐ | ⚠️ OK |
| Manuálna sync | ⭐⭐ | ⭐⭐ | ⚠️ Pomalšie |

**Odporúčanie:** Použite **VS Code Remote SSH** pre každodenný vývoj!
