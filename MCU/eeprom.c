#include <xc.h>
#include "eeprom.h"

// Function to write a 32-bit signed value to EEPROM
void EEPROM_Write_32Bit(unsigned char address, long data) {
    unsigned char i;
    for (i = 0; i < 4; i++) {
        while (EECON1bits.WR);       // Wait for any previous write to complete
        EEADR = address + i;         // Set EEPROM address (incremented for each byte)
        EEDATA = (data >> (i * 8)) & 0xFF;  // Extract each byte from the 32-bit value
        EECON1bits.EEPGD = 0;        // Point to EEPROM
        EECON1bits.CFGS = 0;         // Access EEPROM
        EECON1bits.WREN = 1;         // Enable write operations

        // Critical sequence to begin write
        INTCONbits.GIE = 0;          // Disable global interrupts
        EECON2 = 0x55;               // Required sequence
        EECON2 = 0xAA;               // Required sequence
        EECON1bits.WR = 1;           // Start the write
        INTCONbits.GIE = 1;          // Re-enable global interrupts

        while (EECON1bits.WR);       // Wait for write to complete
        EECON1bits.WREN = 0;         // Disable write operations
    }
}

// Function to read a 32-bit signed value from EEPROM
long EEPROM_Read_32Bit(unsigned char address) {
    unsigned char i;
    long data = 0;

    for (i = 0; i < 4; i++) {
        EEADR = address + i;         // Set EEPROM address (incremented for each byte)
        EECON1bits.EEPGD = 0;        // Point to EEPROM
        EECON1bits.CFGS = 0;         // Access EEPROM
        EECON1bits.RD = 1;           // Start the read
        data |= ((long)EEDATA) << (i * 8); // Combine bytes into a 32-bit value
    }
    
    return data;
}