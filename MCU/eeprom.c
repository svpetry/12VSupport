#include <xc.h>
#include "base.h"
#include "eeprom.h"

// Function to write a 32-bit signed value to EEPROM
void EEPROM_Write_32Bit(unsigned char address, long data) {
    unsigned char i;
    for (i = 0; i < 4; i++) {
        EECON1bits.EEPGD = 0;        // Point to EEPROM
        EECON1bits.CFGS = 0;         // Access EEPROM

        EEADR = address + i;         // Set EEPROM address (incremented for each byte)
        EEDATA = (data >> (i * 8)) & 0xFF;  // Extract each byte from the 32-bit value

        // Critical sequence to begin write
        INTCONbits.GIE = 0;          // Disable global interrupts

        EECON1bits.WREN = 1;         // Enable write operations
        EECON2 = 0x55;               // Required sequence
        EECON2 = 0xAA;               // Required sequence
        EECON1bits.WR = 1;           // Start the write
        while (EECON1bits.WR);       // Wait the write to complete

        EECON1bits.WREN = 0;         // Disable write operations
        INTCONbits.GIE = 1;          // Re-enable global interrupts
    }
}

// Function to read a 32-bit signed value from EEPROM
long EEPROM_Read_32Bit(unsigned char address) {
    unsigned char i;
    long data = 0;

    EECON1bits.WREN = 0;
    for (i = 0; i < 4; i++) {
        EECON1bits.EEPGD = 0;        // Point to EEPROM
        EECON1bits.CFGS = 0;         // Access EEPROM

        EEADR = address + i;         // Set EEPROM address (incremented for each byte)

        EECON1bits.RD = 1;           // Start the read
        __delay_us(1);
        data |= ((long)EEDATA) << (i * 8); // Combine bytes into a 32-bit value
    }
    
    return data;
}