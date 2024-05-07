#include <Arduino.h>
#include <Wire.h>


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


class TempSensor
{
    private:
        TwoWire *I2C;

        // Calibration Constants
        uint16_t PTAT_25, M, U0; 
        float k;

        uint8_t slave_address;

        void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t *dest);
        void writeByte(uint8_t address, uint8_t subAddress, uint8_t data);
        void readEEPROM();
        void wake();
        uint16_t getTPAMB();
        uint32_t getTPOBJ();


    public:
        //
        // Create a TempSensor object 
        //
        TempSensor(uint8_t address, TwoWire &bus);

        //
        // Reads the value of the temperature sensor, and returns in Kelvin
        //
        float Read();
};