# AGENTS.md

Guidance for AI coding agents working in this MCU firmware project.

## Project Overview

This directory contains the PIC18F2480 firmware for the 12V support controller. It is an MPLAB X / XC8 C project generated around a `default` configuration.

The firmware monitors system voltage, battery voltage, battery temperature, and battery current; manages charge/discharge relays, heater, fan, and status LEDs; stores calibration factors in EEPROM; and estimates battery state of charge.

## Important Files

- `main.c`: MCU initialization, pin mapping, Timer1 interrupt, and main charge/discharge state machine.
- `sensors.c` / `sensors.h`: ADC reads, moving-average sensor cache, voltage/current/temp scaling, and calibration.
- `capacity.c` / `capacity.h`: remaining capacity integration, SOC calculation, and voltage-to-SOC lookup tables.
- `eeprom.c` / `eeprom.h`: PIC EEPROM 32-bit read/write helpers.
- `constants.h`: battery chemistry selection and threshold constants.
- `config.h`: PIC18F2480 configuration bits.
- `base.h`: shared MCU timing constant, currently `_XTAL_FREQ`.
- `Makefile`: project-level MPLAB make entrypoint.
- `nbproject/`: generated MPLAB X make/project metadata. Treat these as generated unless deliberately changing IDE configuration.
- `build/`, `dist/`, `debug/`: generated build artifacts.

## Build

Use the project makefile from this directory:

```powershell
make build
```

Other useful targets:

```powershell
make clean
make clobber
make all
```

The generated configuration targets PIC18F2480 and XC8. The current generated metadata references XC8 v2.05 and the Microchip PIC18Fxxxx DFP pack. Production output is expected at:

```text
dist/default/production/MCU.production.hex
```

If command-line builds fail because `make`, `xc8-cc`, or Microchip helper tools are missing from `PATH`, verify the build in MPLAB X instead of rewriting generated makefiles.

## Firmware Architecture Notes

- `main.c` uses Timer1 overflow to run `MainLoop()` on a 0.5-second cadence. The `sec` flag toggles each time and is also used for blinking LEDs.
- Pin assignments are documented at the top of `main.c`; update that table whenever changing hardware-facing port usage.
- The state machine uses `STATE_INITIAL`, `STATE_READY`, `STATE_SUPPLYING`, `STATE_EMPTY`, `STATE_CHARGING`, `STATE_FULL`, and `STATE_OVERHEAT` constants in `main.c`.
- Sensor units are important:
  - `system_voltage`: millivolts.
  - `batt_voltage`: millivolts.
  - `batt_temp`: tenths of degrees Celsius.
  - `batt_current`: milliamps.
  - `batt_current_abs`: absolute milliamps.
  - `full_cap`: milliamp-hours.
  - `rem_cap`: microamp-hours.
  - `soc`: percent-like integer from `rem_cap / (10 * full_cap)`.
- `constants.h` selects the battery chemistry with `#define LIFEPO` or `#define LIION`. Keep only the intended chemistry active.
- `ReadSensors()` updates moving averages sized by `SENSOR_MEM_COUNT`; `FlushSensorCache()` in `main.c` warms that cache after startup or calibration.
- Calibration is triggered by `RA4 == 0` at startup and expects exactly 12V on the supply line.

## Coding Conventions

- Keep code C99-compatible for XC8.
- Prefer the existing simple module style: globals declared in `.c` files and exposed via `extern` in matching headers where needed.
- Use fixed-width integer types where the code already does, but be mindful that many existing calculations use `long` to avoid overflow.
- Preserve hardware units in comments when changing thresholds, scaling formulas, or capacity calculations.
- Avoid dynamic allocation and library features that are inappropriate for small PIC firmware.
- Keep interrupt work short. The high-priority ISR currently reloads Timer1 and calls `MainLoop()` only on the 0.5-second cadence.
- Be careful changing EEPROM writes: `EEPROM_Write_32Bit()` disables global interrupts during the required unlock/write sequence.

## Editing Rules For Agents

- Do not hand-edit generated `nbproject/Makefile-*.mk` files unless the task is explicitly about MPLAB project configuration.
- Do not commit or clean generated artifact directories (`build/`, `dist/`, `debug/`) unless explicitly requested.
- Before changing hardware behavior, inspect both `main.c` pin comments and `constants.h` thresholds.
- Keep edits narrowly scoped; this project has limited safety margin because changes directly control relays, charging, heating, and fan output.
- Do not revert unrelated working-tree changes. This repository may include board files and generated MPLAB metadata modified outside the MCU firmware work.

## Verification

After code changes, prefer:

```powershell
make build
```

If build tools are unavailable locally, state that clearly and review the affected code paths manually. For behavior changes, include the relevant hardware assumptions in the final response, especially voltage thresholds, relay polarity, temperature limits, and calibration expectations.
