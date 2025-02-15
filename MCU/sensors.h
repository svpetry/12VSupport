#ifndef SENSORS_H
#define	SENSORS_H

// sensor values
extern long system_voltage; // system voltage in mV
extern long batt_voltage; // battery voltage in mV
extern int batt_temp; // battery temperature in 1/10 deg C
extern long batt_current; // battery current in mA
extern long batt_current_abs; // abs battery current in mA

void ReadSensors(void);
void InitSensors(void);
void Calibrate(void);

#endif	/* SENSORS_H */

