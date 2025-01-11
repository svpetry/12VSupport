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
 * RC1: EN_BOOST_IN (output)
 * RC2: REL_BUCK_OUT (output)
 * RC3: REL_HEATER (output)
 * RC4: REL_BOOST_IN (output)
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
#include "capacity.h"

// Calculate Timer1 initial value for 0.1-second overflow
// Timer tick period = (Prescaler * 4) / Fosc = (8 * 4) / 10MHz = 3.2 µs
// Counts needed for 0.1 s: 0.1 s / 3.2 µs = 31,250 counts
// Initial value = 65,536 - 31,250 = 34,286
#define TIMER_HI 0x85
#define TIMER_LO 0xEE

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
    LATB = 0;
    TRISC = 0x00;
    LATC = 0;
    TRISA = 0xFF; // PORTA = input
    
    // Configure ADC
    ADCON0 = 0b00000001; // Enable ADC and select channel 0 (AN0)
    ADCON1 = 0b00001011; // Configure AN0 to AN3 as analog inputs, others as digital
    ADCON2 = 0b10101001; // Right justified, 12 TAD, Fosc/8

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

void MainLoop() {
    if (!mainloop_enabled) return;
    
    ReadSensors();
    
    if (state != STATE_INITIAL) {
        if (sec)
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
            LATCbits.LATC6 = 0; // red LED
            LATCbits.LATC7 = 0; // green LED
            
            if (system_voltage < VOLTAGE_START_SUPPLY)
                state = STATE_SUPPLYING;
            else if (system_voltage > VOLTAGE_CHARGE_IF_NEEDED)
                state = STATE_CHARGING;
            else if (batt_voltage <= BAT_VOLTAGE_EMPTY)
                state = STATE_EMPTY;
            break;
        }

        case STATE_SUPPLYING: {
            unsigned char stop_supply = 0;
            LATCbits.LATC7 = 1; // green LED
            LATCbits.LATC2 = 0; // buck output relay

            if (batt_voltage > VOLTAGE_CHARGE_IF_NEEDED) {
                stop_supply = 1;
                state = STATE_CHARGING;
            } else if (batt_voltage < BAT_VOLTAGE_EMPTY) {
                stop_supply = 1;
                state = STATE_EMPTY;
            }
            
            if (stop_supply) {
                LATCbits.LATC7 = 0; // green LED
                LATCbits.LATC2 = 1; // buck output relay
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
                    LATCbits.LATC1 = 1; // boost converter input MOSFET
                    __delay_ms(100);
                    LATCbits.LATC4 = 1; // boost converter input relay
                    __delay_ms(200);
                    LATCbits.LATC0 = 1; // charging relay
                }
            } else {
                LATCbits.LATC6 = sec; // red LED
                LATCbits.LATC1 = 0; // boost converter input MOSFET
                LATCbits.LATC4 = 0; // boost converter input relay
                LATCbits.LATC0 = 0; // charging relay
            }

            unsigned char stop_charge = 0;
            if (system_voltage < VOLTAGE_STOP_CHARGING) {
                stop_charge = 1;
                state = STATE_READY;
            } else if (batt_current < BAT_MIN_CHARGE_CURRENT && batt_voltage > BAT_VOLTAGE_FULL) {
                stop_charge = 1;
                state = STATE_FULL;
            }
            if (stop_charge)
            {
                LATCbits.LATC6 = 0; // red LED
                LATCbits.LATC1 = 0; // boost converter input MOSFET
                LATCbits.LATC4 = 0; // boost converter input relay
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
            if (mainloop_enabled)
                MainLoop();
        }
    }
}

void main(void) {
    
    Initialize();
    
    LATCbits.LATC6 = 1;
    LATCbits.LATC7 = 1;
    __delay_ms(1000);
    LATCbits.LATC6 = 0;
    LATCbits.LATC7 = 0;
    
    // enable main loop
    mainloop_enabled = 1;
    
    while (1) ;
}
