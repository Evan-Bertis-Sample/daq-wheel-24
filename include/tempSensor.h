#include <Arduino.h>

// CaliPile Registers
#define CALIPILE_TPOBJECT 1
#define CALIPILE_TPAMBIENT 3

// EEPROM Registers
#define CALIPILE_EEPROM_CONTROL 31
#define CALIPILE_EEPROM_PTAT25 42
#define CALIPILE_EEPROM_M 44
#define CALIPILE_EEPROM_U0 46
#define CALIPILE_EEPROM_UOUT1 48
#define CALIPILE_EEPROM_TOBJ1 50

// Calipile Address
#define CALIPILE_ADDRESS 0x0C

class TempSensor
{
    public:
        //
        // Create a TempSensor object 
        //
        TempSensor();

        //
        // Reads the value of the temperature sensor, and returns in Kelvin
        //
        float Read();
};