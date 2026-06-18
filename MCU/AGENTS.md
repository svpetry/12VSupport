# AGENTS.md

PIC18F2480 firmware (XC8 / MPLAB X) for the 12V support controller. MPLAB project is `12VSupport` with a single `default` config and build artifact `MCU`; this `MCU/` directory holds the source.

## What it controls

Bridges a 12V system/supply rail (`system_voltage`, ~10–15V) and a 24V battery pack (`batt_voltage`, 20–28V for the default 8s LiFePO4). High rail → boost to charge the battery; low rail → buck the battery down to supply the rail. Outputs: charge/discharge relays, heater, fan, SOC LEDs, I2C OLED.

## Units — read before touching sensor/capacity math

- `system_voltage` mV (12V rail); `batt_voltage` mV (24V pack); `batt_temp` 0.1 °C; `batt_current` / `batt_current_abs` mA.
- `full_cap` mAh; `rem_cap` uAh; `soc` = percent (0–100) via `rem_cap / (10 * full_cap)` — rem_cap is uAh and full_cap mAh, so the raw ratio is ×1000 and the `/10` turns it into percent (easy to break).
- Sensor/capacity values are `long` throughout to avoid overflow in the fixed-point math.

## Build

`make build` (also `clean`, `clobber`, `all`) → `dist/default/production/MCU.production.hex`.

Toolchain/target are in `nbproject/configurations.xml` (XC8 v3.00, PIC18F2480). `nbproject/`, `build/`, `dist/`, `debug/` are generated — don't hand-edit or commit them; if `make` / `xc8-cc` aren't on `PATH`, build in MPLAB X rather than touching the generated makefiles.

## Project-specific gotchas

- **Runtime is interrupt-driven.** After setup, `main()` just spins in `while(1);`. `MainLoop()` and the whole state machine run from the high-priority Timer1 ISR (~every 0.5s, gated by the `sec` flag) — so that code path must stay bounded.
- **EEPROM writes disable interrupts and block.** `EEPROM_Write_32Bit()` clears GIE for the unlock/write sequence. They currently only happen in the pre-loop calibration path; don't add them into the state machine (which is ISR context).
- **Calibration requires external setup.** Triggered by `RA4` low at startup; it expects exactly 12V on the supply line, writes correction factors to EEPROM, then halts forever (needs a reset).
- **Chemistry is a compile-time switch.** `constants.h` selects the pack via `#define LIFEPO` or `#define LIION` — keep exactly one active; the voltage→SOC tables and thresholds differ per chemistry.
- **Safety-sensitive.** This switches real power hardware on a 24V battery; wrong charge/temperature thresholds or relay on/off polarity can overcharge or overheat the pack. `constants.h` thresholds and relay polarity are the load-bearing values to get right.
- This repo also has non-firmware board files and generated metadata that change outside `MCU/`; don't revert unrelated working-tree changes.

## Files

`main.c` (init, pins, Timer1 ISR, state machine) · `sensors.c/h` (ADC + scaling + calibration) · `capacity.c/h` (SOC + voltage→SOC tables) · `eeprom.c/h` (32-bit EEPROM R/W) · `oled.c/h` (SSD1306 over I2C, RC3/RC4) · `constants.h` (chemistry + thresholds) · `config.h` (config bits) · `base.h` (`_XTAL_FREQ` = 10 MHz).
