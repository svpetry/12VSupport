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
 * RA4: CALIBRATE switch
 * RA5: NC
 * RA6: OSC2
 * RA7: OSC1
 * RC0: REL_CHARGE
 * RC1: NC
 * RC2: REL_BUCK_OUT
 * RC3: REL_HEATER
 * RC4: REL_BOOST_IN
 * RC5: NC
 * RC6: LED discharge
 * RC7: LED charge
 * RE3: VPP
 * RB0: SOC LED 20%
 * RB1: SOC LED 40%
 * RB2: SOC LED 60%
 * RB3: SOC LED 80%
 * RB4: SOC LED 100%
 * RB5: FAN
 * RB6: PGC
 * RB7: PGD
 */

#include <xc.h>
#include <pic18f2480.h>
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
#define STATE_OVERHEAT 6

unsigned char mainloop_enabled = 0;
unsigned char state = STATE_INITIAL ;
unsigned char cap_reset_empty = 0;
unsigned char sec = 0;
unsigned char wait = 0;
unsigned char cal_countdown = 10;

void Initialize() {
    // configure ports
    TRISB = 0x00;
    LATB = 0x00;
    TRISC = 0x00;
    LATC = 0x00;
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
    unsigned char charging = LATCbits.LATC7; // charging led
    unsigned char led1 = 0, led2 = 0, led3 = 0, led4 = 0, led5 = 0;
    
    if (soc > 10) led1 = 1;
    if (soc > 30) led2 = 1;
    if (soc > 50) led3 = 1;
    if (soc > 70) led4 = 1;
    if (soc > 90) led5 = 1;

    if (charging) {
        if (!led1)
            led1 = sec;
        else if (!led2)
            led2 = sec;
        else if (!led3)
            led3 = sec;
        else if (!led4)
            led4 = sec;
        else if (!led5)
            led5 = sec;
    }

    LATBbits.LATB0 = led1;
    LATBbits.LATB1 = led2;
    LATBbits.LATB2 = led3;
    LATBbits.LATB3 = led4;
    LATBbits.LATB4 = led5;
}

void FlushSensorCache() {
    for (unsigned char i = 0; i < SENSOR_MEM_COUNT; i++) {
        ReadSensors();
        __delay_ms(10);
    }
}

void SwitchState(unsigned char new_state) {
    state = new_state;
    wait = 8;
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
    
    if (wait > 0) {
        wait--;
        return;
    }

    switch (state) {
        case STATE_INITIAL: {
            LATC = 0b00000100;
            LATB = 0;
            if (batt_voltage > BATT_VOLTAGE_MIN) {
                FlushSensorCache();
                full_cap = INITIAL_FULL_CAP;
                rem_cap = GuessRemainingCap();
                SwitchState(STATE_READY);
            } else {
                // no battery
                LATCbits.LATC6 = sec; // discharge LED
                LATCbits.LATC7 = 1 - sec; // charge LED
            }
            break;
        }
        
        case STATE_READY: {
            LATCbits.LATC6 = 1; // discharge LED
            LATCbits.LATC7 = 0; // charge LED
            __delay_ms(20);
            LATCbits.LATC6 = 0; // discharge LED
            
            if (batt_voltage >= BATT_VOLTAGE_MIN) {
                if (system_voltage <= SYS_VOLTAGE_START_SUPPLY)
                    SwitchState(STATE_SUPPLYING);
                else if (system_voltage >= SYS_VOLTAGE_START_CHARGE)
                    SwitchState(STATE_CHARGING);
                else if (batt_voltage <= BAT_VOLTAGE_EMPTY)
                    SwitchState(STATE_EMPTY);
            }
            break;
        }

        case STATE_SUPPLYING: {
            unsigned char stop_supply = 0;
            
            LATCbits.LATC6 = 1; // discharge LED
            if (batt_temp <= MAX_TEMP)
                LATCbits.LATC2 = 0; // buck output relay (1 = off)
            else
                LATCbits.LATC2 = 1; // buck output relay (1 = off)

            if (system_voltage >= SYS_VOLTAGE_STOP_SUPPLY) {
                stop_supply = 1;
                wait = 4;
                SwitchState(STATE_READY);
            } else if (batt_voltage <= BAT_VOLTAGE_EMPTY) {
                stop_supply = 1;
                SwitchState(STATE_EMPTY);
            }
            
            if (stop_supply) {
                LATCbits.LATC6 = 0; // discharge LED
                LATCbits.LATC2 = 1; // buck output relay (1 = off)
            }
            
            break;
        }

        case STATE_EMPTY: {
            if (!cap_reset_empty) {
                full_cap = full_cap - rem_cap / 1000;
                if (full_cap < MIN_CAP)
                    full_cap = MIN_CAP;
                rem_cap = 0;
                cap_reset_empty = 1;
            }
            LATCbits.LATC6 = sec; // discharge LED
            if (system_voltage >= SYS_VOLTAGE_START_CHARGE) {
                LATCbits.LATC6 = 0; // discharge LED
                SwitchState(STATE_CHARGING);
            }
            break;
        }

        case STATE_CHARGING: {
            if (batt_temp < HEATER_TEMP)
                LATCbits.LATC3 = 1; // heater relay
            else 
                LATCbits.LATC3 = 0; // heater relay
            
            static unsigned char charging = 0;
            if ((charging || batt_temp >= CHARGING_MIN_TEMP) && batt_temp <= MAX_TEMP) {
                if (!charging) {
                    LATCbits.LATC7 = 1; // charge LED
                    LATBbits.LATB5 = 1; // fan
                    LATCbits.LATC4 = 1; // boost converter input relay
                    __delay_ms(250);
                    charging = 1;
                }
            } else {
                LATCbits.LATC7 = sec; // charge LED
                LATBbits.LATB5 = 0; // fan
                LATCbits.LATC4 = 0; // boost converter input relay
                LATCbits.LATC0 = 0; // charging relay
                charging = 0;
            }

            if (charging && !LATCbits.LATC0) {
                LATCbits.LATC0 = 1; // charging relay
                wait = 6;
                return;
            }
            
            unsigned char stop_charge = 0;
            if (system_voltage <= SYS_VOLTAGE_STOP_CHARGE) {
                stop_charge = 1;
                SwitchState(STATE_READY);
            } else if (charging && batt_current_abs <= BAT_MIN_CHARGE_CURRENT) {
                if (batt_voltage >= BAT_VOLTAGE_FULL) {
                    stop_charge = 1;
                    SwitchState(STATE_FULL);
                } else {
                    // workaround for faulty songle relais
                    LATCbits.LATC0 = 0; // charging relay
                    wait = 4;
                    return;
                }
            }
            if (stop_charge)
            {
                charging = 0;
                LATCbits.LATC7 = 0; // charge LED
                LATBbits.LATB5 = 0; // fan
                LATCbits.LATC4 = 0; // boost converter input relay
                LATCbits.LATC0 = 0; // charging relay
                LATCbits.LATC3 = 0; // heater relay
            }
            break;
        }

        case STATE_FULL: {
            if (cap_reset_empty) {
                cap_reset_empty = 0;
                full_cap = rem_cap / 1000;
                if (full_cap < MIN_CAP)
                    full_cap = MIN_CAP;
            } else
                rem_cap = full_cap * 1000;

            if (system_voltage <= SYS_VOLTAGE_STOP_CHARGE)
                SwitchState(STATE_READY);
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
        TMR1H = TIMER_HI;
        TMR1L = TIMER_LO;

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
    InitSensors();
    
    LATCbits.LATC6 = 1;
    LATCbits.LATC7 = 1;
    __delay_ms(2000);
    LATCbits.LATC6 = 0;
    LATCbits.LATC7 = 0;
    
    LATCbits.LATC2 = 1; // buck output relay (1 = off)
    __delay_ms(1000);
    
    FlushSensorCache();
    
    // calibration
    if (PORTAbits.RA4 == 0) {
        LATB = 0x1F;
        __delay_ms(100);
        LATB = 0x00;
        __delay_ms(100);
        LATB = 0x1F;
        __delay_ms(100);
        LATB = 0x00;
        __delay_ms(100);
        LATB = 0x1F;
        __delay_ms(100);
        LATB = 0x00;

        __delay_ms(100);
        
        Calibrate();
        while (1) ;
    }
    
    __delay_ms(3000);

    // enable main loop
    mainloop_enabled = 1;
    
    while (1) ;
}
