/* 
 * File:   sensors.h
 * Author: svpet
 *
 * Created on 1. Dezember 2024, 19:05
 */

#ifndef SENSORS_H
#define	SENSORS_H

// sensor values
extern long system_voltage; // system voltage in mV
extern long batt_voltage; // battery voltage in mV
extern int batt_temp; // battery temperature in 1/10 deg C
extern long batt_current; // battery current in mA

void ReadSensors(void);

#endif	/* SENSORS_H */

