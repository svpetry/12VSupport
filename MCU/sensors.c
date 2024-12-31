#include <xc.h>
#include "sensors.h"
#include "base.h"

// sensor values
long system_voltage; // system voltage in mV
long batt_voltage; // battery voltage in mV
int batt_temp; // battery temperature in 1/10 deg C
long batt_current; // battery current in mA

unsigned int ADC_Read(unsigned char channel) {
    if (channel > 13) return 0;  // Invalid channel check
    
    ADCON0 &= 0b11000001;        // Clear channel selection bits
    ADCON0 |= (channel << 2);    // Set the required channel
    __delay_us(10);              // Acquisition time to charge hold capacitor
    
    ADCON0bits.GO = 1;           // Start A/D conversion
    while (ADCON0bits.GO);       // Wait for conversion to complete
    
    return (unsigned int)((ADRESH << 8) + ADRESL);  // Return 10-bit result
}

long ReadMillivolts(unsigned char channel) {
    long value = ADC_Read(channel);
    return (value * 5 * 1000) / 1024;
}

void ReadSensors() {
    // voltage is divided by 4
    system_voltage = ReadMillivolts(0) * 4;
    
    // 10 mV/degC, 500 mV = 0 degC, multiplied with 4
    batt_temp = (int)((ReadMillivolts(1) - 4 * 500) / 4);
    
    // voltage is divided by 10
    batt_voltage = ReadMillivolts(2) * 10;
    
    // 100 mV/A
    batt_current = (ReadMillivolts(3) - 2500) * 10;
}
