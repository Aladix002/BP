# Motor (jeden uzol)

Autonómna navigácia (Nav2, `cmd_vel`): pozri [NAV_AUTONOMY.md](NAV_AUTONOMY.md).

---

Spustenie: `ros2 run waverower waverower`

- **Uzol:** `wasd_motor_hat_node` — klávesnica (TTY), **alebo** `geometry_msgs/Twist` na **`manual_twist_topic`** (predvolene `/teleop_cmd_vel`) z PC, **alebo** v auto režime `cmd_vel` (Nav2) + HAT (I2C) v jednom procese.
- **Prepínač:** parameter `control_mode` = `manual` \| `auto` (za behu: `ros2 param set ...`, z TTY aj kláves **M**). Detail: [NAV_AUTONOMY.md](NAV_AUTONOMY.md).
- **Kód:** `wasd_motor_hat.*` + `motor_hat_i2c.*` (I2C vrstva).

Predvolené parametre sú v `main.cpp` (`parameter_overrides`), prípadne:

```bash
ros2 run waverower waverower --ros-args -p i2c_bus:=10
```

Pri **plnom** W už ide PWM na 100 % — ďalšie zrýchlenie je hlavne **batéria / motory**. Nižší `snap_threshold` = skôr „plný plyn“ pri menšom príkaze (agresívnejšie).

- **W / S** a **A / D** (rovnako šípky) majú znamienka prispôsobené montáži motora; úprava je v `input_loop` v `wasd_motor_hat.cpp`.
- **Otáčanie na mieste** má predvolene **rovnaký snap ako vpred** (`turn_snap_threshold` = 0 = použiť `snap_threshold`) → plný PWM do kolies ako pri držaní W. `smooth_alpha_spin` (predvolene 1.0) urýchľuje nábeh L/R pri čistom otočení, aj keď je globálne `smooth_alpha` menšie. Lineárne „jemné“ otáčanie: `-p turn_snap_threshold:=1.0`.
