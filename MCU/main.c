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

#include <xc.h>
#include "config.h"
#include "constants.h"
#include "base.h"
#include "eeprom.h"
#include "sensors.h"

// Calculate Timer1 initial value for 0.1-second overflow
// Timer tick period = (Prescaler * 4) / Fosc = (8 * 4) / 10MHz = 3.2 µs
// Counts needed for 0.1 s: 0.1 s / 3.2 µs = 31,250 counts
// Initial value = 65,536 - 31,250 = 34,286
#define TIMER_HI 0x85
#define TIMER_LO 0xEE

// capacity values
long rem_cap; // remaining capacity in uAh
long full_cap; // full capacity in mAh
int soc = 0; // current calculated SOC

typedef struct {
    long voltage; // voltage in mV
    int soc; // SOC in permille
} VoltageSoc;

int soc_level_count = 11;
VoltageSoc voltage_soc[] = 
{
    {28800, 1000},
    {28000, 900},
    {27200, 800},
    {26800, 700},
    {26560, 600},
    {26400, 500}, 
    {26240, 400},
    {26080, 300},
    {25600, 200},
    {24800, 100},
    {20000, 0}
};

// program flow
#define STATE_INITIAL 0
#define STATE_READY 1
#define STATE_SUPPLYING 2
#define STATE_EMPTY 3
#define STATE_CHARGING 4
#define STATE_FULL 5
unsigned char mainloop_enabled = 0;
unsigned char state = STATE_INITIAL ;
unsigned char cap_reset_empty = 0;
unsigned char sec = 0;

void Initialize() {
    // configure ports
    TRISB = 0x00;
    TRISC = 0x00;
    
    // Configure ADC
    ADCON0 = 0x01;  // Enable ADC and select channel 0 (AN0)
    ADCON1 = 0x09;  // Configure AN0 to AN3 as analog inputs, others as digital
    ADCON2 = 0xA9;  // Right justified, 12 TAD, Fosc/8

    // Set up Timer1 for 0.1-second overflows
    T1CONbits.TMR1CS = 0;    // Clock source = internal (Fosc/4)
    T1CONbits.T1CKPS = 0b11; // Prescaler = 1:8
    T1CONbits.T1RD16 = 1;    // Enable 16-bit Read/Write
    T1CONbits.TMR1ON = 1;    // Turn on Timer1

    TMR1H = TIMER_HI; // High byte of 34,286
    TMR1L = TIMER_LO; // Low byte of 34,286

    // Enable Timer1 interrupt
    PIR1bits.TMR1IF = 0;    // Clear Timer1 interrupt flag
    PIE1bits.TMR1IE = 1;    // Enable Timer1 interrupt

    // Enable global and peripheral interrupts
    INTCONbits.PEIE = 1;    // Enable Peripheral Interrupts
    INTCONbits.GIE = 1;     // Enable Global Interrupts
}

long GuessRemainingCap() {
    long soc_pm = 0; // SOC in permille
    
    if (batt_current >= voltage_soc[0].voltage) {
        soc_pm = voltage_soc[0].soc;
    } else {
        for (int i = 1; i < soc_level_count; i++) {
            if (batt_voltage >= voltage_soc[i].voltage) {
                soc_pm = 
                    (batt_voltage - voltage_soc[i].voltage) 
                    * (voltage_soc[i - 1].soc - voltage_soc[i].soc) 
                    / (voltage_soc[i - 1].voltage - voltage_soc[i].voltage) 
                    + voltage_soc[i].soc;
                break;
            }
        }
    }
    return full_cap * soc_pm;
}

void CountRemainingCapacity() {
    if (!sec) return;
    
    // this is executed every second
    rem_cap -= batt_current * 1000 / 3600;
    if (rem_cap < 0)
        rem_cap = 0;
}

void CalcSoc() {
    soc = (int)(rem_cap / (10 * full_cap));
}

void SetSocLeds() {
    unsigned char portb = 0;
    if (soc > 0)
        portb |= 0x01;
    if (soc > 20)
        portb |= 0x02;
    if (soc > 40)
        portb |= 0x04;
    if (soc > 60)
        portb |= 0x08;
    if (soc > 80)
        portb |= 0x10;
    LATB = portb;
}

long InitFullCap() {
    if (EEPROM_Read_32Bit(EEPROM_ADDR_VERSION) == EEPROM_VERSION) 
        return EEPROM_Read_32Bit(EEPROM_ADDR_FULL_CAP);
    return INITIAL_FULL_CAP;
}

void SetFullCap(long value) {
    full_cap = value;
    EEPROM_Write_32Bit(EEPROM_ADDR_VERSION, EEPROM_VERSION);
    EEPROM_Write_32Bit(EEPROM_ADDR_FULL_CAP, value);
}

void MainLoop() {
    if (!mainloop_enabled) return;
    
    ReadSensors();
    
    if (state != STATE_INITIAL) {
        CountRemainingCapacity();
        CalcSoc();
        SetSocLeds();
    }
    
    switch (state) {
        case STATE_INITIAL: {
            LATC = 0;
            LATB = 0;
            if (batt_voltage > BATT_MIN_VOLTAGE) {
                full_cap = InitFullCap();
                rem_cap = GuessRemainingCap();
                state = STATE_READY;
            } else {
                // no battery
                LATCbits.LATC6 = sec; // red LED
                LATCbits.LATC7 = 1 - sec; // green LED
            }
            break;
        }
        
        case STATE_READY: {
            if (system_voltage < VOLTAGE_START_SUPPLY) {
                LATCbits.LATC6 = 0; // red LED
                state = STATE_SUPPLYING;
            } else if (system_voltage > VOLTAGE_CHARGE_IF_NEEDED) {
                LATCbits.LATC6 = 0; // red LED
                state = STATE_CHARGING;
            } else if (batt_voltage <= BAT_VOLTAGE_EMPTY)
                LATCbits.LATC6 = sec; // red LED
            break;
        }

        case STATE_SUPPLYING: {
            unsigned char stop_supply = 0;
            LATCbits.LATC7 = 1; // green LED
            LATCbits.LATC2 = 1; // buck output relay

            if (batt_voltage > VOLTAGE_CHARGE_IF_NEEDED) {
                stop_supply = 1;
                state = STATE_CHARGING;
            } else if (batt_current < BAT_CURRENT_THRESHOLD && batt_voltage < BAT_VOLTAGE_EMPTY) {
                stop_supply = 1;
                state = STATE_EMPTY;
            }
            
            if (stop_supply) {
                LATCbits.LATC7 = 0; // green LED
                LATCbits.LATC2 = 0; // buck output relay
            }
            
            break;
        }

        case STATE_EMPTY: {
            if (!cap_reset_empty) {
                SetFullCap(full_cap - rem_cap / 1000);
                rem_cap = 0;
                cap_reset_empty = 1;
            }
            LATCbits.LATC7 = sec; // green LED
            if (batt_voltage > VOLTAGE_CHARGE_IF_NEEDED) {
                LATCbits.LATC7 = 0; // green LED
                state = STATE_CHARGING;
            }
            break;
        }

        case STATE_CHARGING: {
            if (batt_temp < HEATER_TEMP)
                LATCbits.LATC3 = 1; // heater relay
            else 
                LATCbits.LATC3 = 0; // heater relay
            
            if (batt_temp >= CHARGING_MIN_TEMP && 
                    (system_voltage > VOLTAGE_FORCE_CHARGING || soc < SOC_CHARGE_IF_NEEDED)) {
                if (!PORTCbits.RC0) {
                    LATCbits.LATC6 = 1; // red LED
                    LATCbits.LATC1 = 1; // boost converter input relay
                    __delay_ms(200);
                    LATCbits.LATC0 = 1; // charging relay
                }
            } else {
                LATCbits.LATC6 = sec; // red LED
                LATCbits.LATC1 = 0; // boost converter input
                LATCbits.LATC0 = 0; // charging relay
            }

            unsigned char stop_charge = 0;
            if (system_voltage < VOLTAGE_STOP_CHARGING) {
                stop_charge = 1;
                state = STATE_READY;
            } else if (batt_current < BAT_CURRENT_THRESHOLD && batt_voltage > BAT_VOLTAGE_FULL) {
                stop_charge = 1;
                state = STATE_FULL;
            }
            if (stop_charge)
            {
                LATCbits.LATC6 = 0; // red LED
                LATCbits.LATC1 = 0; // boost converter input
                LATCbits.LATC0 = 0; // charging relay
                LATCbits.LATC3 = 0; // heater relay
            }
            break;
        }

        case STATE_FULL: {
            if (cap_reset_empty) {
                cap_reset_empty = 0;
                SetFullCap(rem_cap / 1000);
            } else
                rem_cap = full_cap * 1000;

            if (system_voltage < VOLTAGE_STOP_CHARGING)
                state = STATE_READY;
            break;
        }

    }
}

// Interrupt Service Routine
void __interrupt(high_priority) HighISR(void) {
    // Check for Timer1 overflow interrupt
    if (PIR1bits.TMR1IF) {
        // Clear the Timer1 interrupt flag
        PIR1bits.TMR1IF = 0;

        // Reload Timer1 register for next interrupt
        TMR1H = TIMER_HI; // High byte of 34,286
        TMR1L = TIMER_LO; // Low byte of 34,286

        // Increment overflow counter
        static unsigned char overflow_count = 0;
        overflow_count++;
        if (overflow_count >= 5) {  // 0.1 s * 5 = 0.5 s
            overflow_count = 0;
            sec ^= 1;
            MainLoop();
        }
    }
}

void main(void) {
    
    Initialize();
    LATCbits.LATC6 = 1;
    LATCbits.LATC7 = 1;

    __delay_ms(3000);
    
    // enable main loop
    mainloop_enabled = 1;
    
    while (1) ;
}
