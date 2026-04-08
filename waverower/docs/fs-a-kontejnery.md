# Súborový systém (read-only) a kontajnery — čo je v projekte a čo riešiť na OS

Tento dokument zodpovedá požiadavke na odolnosť voči výpadku napájania (neočakované vypnutie bez `sync`) a na voliteľný beh v kontajneroch.

## Čo už máte v tomto repozitári (nie je to read-only root)

1. **Jednotný ROS 2 doménový priestor**  
   V launch súboroch (`manual_bringup`, `runtime_stack`, `slam`, `wander`, `web_teleop_ui`) a v `systemd/*.service` je nastavené `ROS_DOMAIN_ID=0`.  
   **Účel:** druhý počítač (napr. notebook s RViz) vidí rovnaké topic-y ako Raspberry Pi, ak má rovnaké `ROS_DOMAIN_ID` a sieť (multicast/firewall podľa vašej inštalácie).

2. **Kontrolované vypnutie z aplikácie**  
   Uzol `robot_mode_switch` expose službu `/waverower/shutdown`, ktorá po ~1 s zavolá `sudo systemctl poweroff`.  
   **Účel:** dať používateľovi cestu na čisté vypnutie pred odpojením napájania.  
   **Obmedzenie:** nezaručí dokončenie zápisu na disk, ak sú iné procesy aktívne; neprepína root FS do read-only.

3. **Systemd služby**  
   `Restart=always` na motorovom uzle znižuje dopad pádu procesu, nie výpadku napájania.

**Zhrnutie:** v kóde **nie je** konfigurácia read-only root, overlayfs ani Docker/Podman. To sú zmeny na úrovni OS / nasadenia.

---

## Read-only alebo overlay root (odporúčané pri výpadkoch napájania)

Cieľ: minimalizovať poškodenie ext4 pri náhlom odpojení SD/eMMC (neukončený journal, polozapísané súbory).

Možnosti na Raspberry Pi / Ubuntu:

| Prístup | Stručný popis |
|--------|----------------|
| **Raspberry Pi OS — „Overlay“** | V `raspi-config` → Performance Options → Overlay File System (read-only root s writable overlay v RAM alebo na oddelenom úseku — závisí od verzie). Po zapnutí treba vedieť, kde sú zapisovateľné dáta (napr. workspace, logy). |
| **Ubuntu + `overlayroot`** | Balík `overlayroot` (ak je dostupný pre vašu verziu) alebo vlastné parametre kernel/cmdline s overlay — root ostane efektívne read-only, zmeny idú do horného writable layer (často tmpfs alebo oddiel). |
| **Tmpfs pre zápisové adresáre** | Napr. `/var/log`, `/tmp` na `tmpfs` zníži zápisy na SD; samo o sebe **nepoistí** root pred poškodením pri blackoute. |
| **Externý SSD + journaling** | Kvalitnejší médiál a UPS sú praktické doplnky k vyššie uvedenému. |

**Workspace (kolcon `install/`, logy):** pri read-only root ich treba mať na zapisovateľnom bind mounte alebo na oddieli, inak nebudete môcť normálne buildiť ani zapisovať logy.

**Čo spraviť pred záverečnou obhajobou / prevádzkou:** na cieľovom Pi si overiť konkrétny postup pre vašu distro verziu (napr. Ubuntu 24.04 na Pi má iné balíky ako Raspberry Pi OS), otestovať reboot a či ROS služby štartujú s vašimi cestami v `waverower.service`.

---

## Kontajnery (Docker / Podman)

V tomto repozitári **nie sú** Dockerfile ani compose súbory — zámerne: robot potrebuje **I²C (Motor HAT), USB sériové porty (LiDAR, IMU), často kameru**, čo v kontajneri znamená typicky:

- `--privileged` alebo veľký zoznam `--device`,
- mapovanie `/dev/ttyACM*`, `/dev/ttyUSB*`, `/dev/i2c-*`,
- a stále zložitejšie ladenie ako natívny ROS 2 na Pi.

**Odporúčané rozdelenie:**

- **Raspberry Pi:** natívna inštalácia ROS 2 + vaše systemd služby (aktuálny stav).
- **Vývojový / silnejší PC:** oficiálne obrazy [Open Robotics / OSRF](https://hub.docker.com/r/osrf/ros) pre build/test bez hardvéru; prípadne len RViz + `ROS_DOMAIN_ID` ako teraz v komentári v `manual_bringup.launch.py`.
- **Izolácia len web/rosbridge:** teoreticky menší kontajner s `rosbridge_server` na PC; robot ostáva zdroj pravdy pre topic-y.

Ak neskôr pridáte Dockerfile, v README popíšte presné `device` / `group_add` pre váš hardvér.

---

## Kontrolný checklist (mimo gitu)

- [ ] Na Pi je zvolená stratégia: overlay / read-only root alebo aspoň minimalizácia zápisov (tmpfs logy).
- [ ] `sudo systemctl poweroff` funguje z účtu, pod ktorým beží ROS (sudoers pre `/waverower/shutdown`).
- [ ] Druhý PC: rovnaký `ROS_DOMAIN_ID`, sieť, overené `ros2 topic list`.
- [ ] Po zapnutí read-only: overené cesty v `ExecStart` a `source .../install/setup.bash`.

---

*Súbor dopĺňa text bakalárskej práce v časti o prevádzkovej spoľahlivosti; samotná záverečná práca môže odkazovať na tento dokument alebo stručne zhrnúť body vyššie.*
