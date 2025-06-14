#ifndef EEPROM_H
#define	EEPROM_H

void EEPROM_Write_32Bit(uint8_t address, long data);
long EEPROM_Read_32Bit(uint8_t address);

#endif	/* EEPROM_H */

