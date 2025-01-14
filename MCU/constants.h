#ifndef CONSTANTS_H
#define	CONSTANTS_H

#define LIFEPO
//#define LIION

#ifdef LIFEPO
// LiFePo4 24V (8s))
#define INITIAL_FULL_CAP 40000
#define BAT_VOLTAGE_EMPTY 20000
#define BAT_VOLTAGE_FULL 28000
#endif

#ifdef LIION
// LiIon 18650 (10S5P)
#define CELL_CAP 2900
#define CELLS_PARALLEL 5
#define CELLS_SERIAL 10
#define INITIAL_FULL_CAP (CELL_CAP * CELLS_PARALLEL)
#define BAT_VOLTAGE_EMPTY (2.5 * CELLS_SERIAL)
#define BAT_VOLTAGE_FULL (4.2 * CELLS_SERIAL)
#endif

#define BATT_MIN_VOLTAGE 5000
#define VOLTAGE_FORCE_CHARGING 14000
#define VOLTAGE_CHARGE_IF_NEEDED 13500
#define SOC_CHARGE_IF_NEEDED 80
#define VOLTAGE_STOP_CHARGING 13200
#define VOLTAGE_START_SUPPLY 12700
#define CHARGING_MIN_TEMP 50
#define HEATER_TEMP 60
#define BAT_MIN_CHARGE_CURRENT 500

#define EEPROM_VERSION 0x5502
#define EEPROM_ADDR_VERSION 0x00
#define EEPROM_ADDR_FULL_CAP 0x04

#endif	/* CONSTANTS_H */

