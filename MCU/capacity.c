#include <xc.h>
#include "capacity.h"
#include "constants.h"
#include "sensors.h"
#include "eeprom.h"

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
    // this is executed every second
    rem_cap -= batt_current * 1000 / 3600;
    if (rem_cap < 0)
        rem_cap = 0;
}

void CalcSoc() {
    soc = (int)(rem_cap / (10 * full_cap));
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
