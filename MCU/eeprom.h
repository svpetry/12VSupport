#ifndef EEPROM_H
#define	EEPROM_H

void EEPROM_Write_32Bit(unsigned char address, long data);
long EEPROM_Read_32Bit(unsigned char address);

#endif	/* EEPROM_H */

