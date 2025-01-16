#include <xc.h>
#include "sensors.h"
#include "eeprom.h"
#include "constants.h"
#include "base.h"

// sensor values
long system_voltage; // system voltage in mV
long batt_voltage; // battery voltage in mV
int batt_temp; // battery temperature in 1/10 deg C
long batt_current; // battery current in mA
long batt_current_abs; // abs battery current in mA

long correction = 0;
long amp_correction = 0;

unsigned int ADC_Read(unsigned char channel) {
    if (channel > 13) return 0;  // Invalid channel check
    
    ADCON0 &= 0b11000001;        // Clear channel selection bits
    ADCON0 |= (channel << 2);    // Set the required channel
    __delay_us(50);              // Acquisition time to charge hold capacitor
    
    ADCON0bits.GO = 1;           // Start A/D conversion
    while (ADCON0bits.GO);       // Wait for conversion to complete
   
    return (unsigned int)((ADRESH << 8) + ADRESL);  // Return 10-bit result
}

long ReadMillivolts(unsigned char channel) {
    long value = 0;
    for (unsigned char i = 0; i < 5; i++) {
        value += ADC_Read(channel);
        __delay_us(50);
    }
    return (value * 1000) / 1024 + correction;
}

void ReadBattCurrent() {
    // 100 mV/A
    batt_current = (ReadMillivolts(3) - 2500) * 10 + amp_correction;
    if (batt_current < 0)
        batt_current_abs = -batt_current;
    else
        batt_current_abs = batt_current;
}

void ReadSensors() {
    // voltage is divided by 4
    system_voltage = ReadMillivolts(0) * 4;
    
    // 10 mV/degC, 500 mV = 0 degC, multiplied with 4
    batt_temp = (int)((ReadMillivolts(1) - 4 * 500) / 4);
    
    // voltage is divided by 10
    batt_voltage = ReadMillivolts(2) * 10;

    ReadBattCurrent();
}

void InitSensors() {
    correction = EEPROM_Read_32Bit(EEPROM_ADDR_CORRECTION_FACTOR);
    amp_correction = EEPROM_Read_32Bit(EEPROM_ADDR_AMP_CORRECTION_FACTOR);
}

void Calibrate() {
    // requires exactly 12V on the supply line
    correction = 0;
    amp_correction = 0;

    correction = 3000 - ReadMillivolts(0);

    ReadBattCurrent();
    amp_correction = -batt_current;
    
    EEPROM_Write_32Bit(EEPROM_ADDR_CORRECTION_FACTOR, correction);
    EEPROM_Write_32Bit(EEPROM_ADDR_AMP_CORRECTION_FACTOR, amp_correction);
}