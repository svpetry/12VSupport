/*
 * File:   main.c
 * Author: svpet
 *
 * Created on 30. November 2024, 10:10
 * 
 * pin layout:
 * RA0: 12V_SENSE (AN0)
 * RA1: TEMP_SENSE (AN1)
 * RA2: BAT_SENSE (AN2)
 * RA3: CURR_SENSE (AN3)
 * RA4: NC
 * RA5: NC
 * RA6: OSC2
 * RA7: OSC1
 * RC0: REL_CHARGE (output)
 * RC1: REL_BOOST_IN (output)
 * RC2: REL_BUCK_OUT (output)
 * RC3: REL_HEATER (output)
 * RC4: NC
 * RC5: NC
 * RC6: LED red
 * RC7: LED green
 * RE3: VPP
 * RB0: SOC LED 20%
 * RB1: SOC LED 40%
 * RB2: SOC LED 60%
 * RB3: SOC LED 80%
 * RB4: SOC LED 100%
 * RB5: NC
 * RB6: PGC
 * RB7: PGD
 */

#include "config.h"
#include "base.h"

#include <xc.h>

#define INITIAL_FULL_CAP 38000

// capacity values
unsigned int rem_cap; // remaining capacity in mAh
unsigned int full_cap; // full capacity in mAh

// sensor values
long system_voltage; // system voltage in mV
long batt_voltage; // battery voltage in mV
int batt_temp; // battery temperature in 1/10 deg C
long batt_current; // battery current in mA

unsigned int GuessRemainingCap() {
    
}

void Initialize() {
    // Configure ADC
    ADCON0 = 0x01;  // Enable ADC and select channel 0 (AN0)
    ADCON1 = 0x09;  // Configure AN0 to AN3 as analog inputs, others as digital
    ADCON2 = 0xA9;  // Right justified, 12 TAD, Fosc/8

    ReadSensors();

    // variables
    full_cap = INITIAL_FULL_CAP;
    rem_cap = GuessRemainingCap();
}

unsigned int ADC_Read(unsigned char channel) {
    if (channel > 13) return 0;  // Invalid channel check
    
    ADCON0 &= 0xC5;              // Clear channel selection bits
    ADCON0 |= (channel << 2);    // Set the required channel
    __delay_us(10);              // Acquisition time to charge hold capacitor
    
    ADCON0bits.GO = 1;           // Start A/D conversion
    while (ADCON0bits.GO);       // Wait for conversion to complete
    
    return (unsigned int)((ADRESH << 8) + ADRESL);  // Return 10-bit result
}

long ReadMillivolts(unsigned char channel) {
    long value = ADC_Read(channel);
    return value * 5 * 1000 / 1024;
}

void ReadSensors() {
    // voltage is divided by 4
    system_voltage = ReadMillivolts(0) * 4;
    
    // 10 mV/degC, 750 mV = 0 degC
    batt_temp = (int)((ReadMillivolts(1) - 4 * 750) / 4);
    
    // voltage is divided by 10
    batt_voltage = ReadMillivolts(2) * 10;
    
    // 100 mV/A
    batt_current = ReadMillivolts(3) * 10;
}

void main(void) {
    
    Initialize();
    
    __delay_ms(3000);
    
    ReadSensors();
    
    return;
}
