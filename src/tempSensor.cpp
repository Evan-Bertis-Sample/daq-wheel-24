#include <Arduino.h>
#include <tempSensor.h>
#include <Wire.h>

// Collect Data
uint16_t Tamb;
uint32_t Tobj;

// Calibration Constants
uint32_t Uout1;
uint8_t Tobj1;
uint16_t PTAT_25, M, U0;
float k;

void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t *dest)
{
    Wire.beginTransmission(address); // Initialize the Tx buffer
    Wire.write(subAddress);          // Put slave register address in Tx buffer
    Wire.endTransmission(false);     // Send the Tx buffer, but send a restart to keep connection alive
    uint8_t i = 0;
    Wire.requestFrom(address, count); // Read bytes from slave register address
    while (Wire.available())
    {
        dest[i++] = Wire.read();
    } // Put read results in the Rx buffer
}
void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
    Wire.beginTransmission(address); // Initialize the Tx buffer
    Wire.write(subAddress);          // Put slave register address in Tx buffer
    Wire.write(data);                // Put data in Tx buffer
    Wire.endTransmission();          // Send the Tx buffer
}
void readEEPROM()
{
    uint8_t rawData[2] = {0, 0};
    writeByte(CALIPILE_ADDRESS, CALIPILE_EEPROM_CONTROL, 0x80);
    // Read calibration constants
    readBytes(CALIPILE_ADDRESS, CALIPILE_EEPROM_PTAT25, 2, &rawData[0]);
    PTAT_25 = ((uint16_t)rawData[0] << 8) | rawData[1];

    readBytes(CALIPILE_ADDRESS, CALIPILE_EEPROM_M, 2, &rawData[0]);
    M = ((uint16_t)rawData[0] << 8) | rawData[1];
    M /= 100;

    readBytes(CALIPILE_ADDRESS, CALIPILE_EEPROM_U0, 2, &rawData[0]);
    U0 = ((uint16_t)rawData[0] << 8) | rawData[1];
    U0 += 32768;

    readBytes(CALIPILE_ADDRESS, CALIPILE_EEPROM_UOUT1, 2, &rawData[0]);
    Uout1 = ((uint16_t)rawData[0] << 8) | rawData[1];
    Uout1 *= 2;

    readBytes(CALIPILE_ADDRESS, CALIPILE_EEPROM_TOBJ1, 1, &Tobj1);

    writeByte(CALIPILE_ADDRESS, CALIPILE_EEPROM_CONTROL, 0x00);

    k = ((float)(Uout1 - U0)) / (powf((float)(Tobj1 + 273.15f), 3.8f) - powf(25.0f + 273.15f, 3.8f));
}
void wake()
{
    writeByte(0x00, 0x04, 0x00); // issue general call and reload command
    delay(1);
}

uint16_t getTPAMB()
{
    uint8_t rawData[2] = {0, 0};
    readBytes(CALIPILE_ADDRESS, CALIPILE_TPAMBIENT, 2, &rawData[0]);
    uint16_t temp = ((uint16_t)(rawData[0] & 0x7F) << 8) | rawData[1];
    return temp;
}
uint32_t getTPOBJ()
{
    uint8_t rawData[3] = {0, 0, 0};
    readBytes(CALIPILE_ADDRESS, CALIPILE_TPOBJECT, 3, &rawData[0]);
    uint32_t temp = ((uint32_t)((uint32_t)rawData[0] << 24) | ((uint32_t)rawData[1] << 16) | ((uint32_t)rawData[2] & 0x80) << 8) >> 15;
    return temp;
}



TempSensor::TempSensor(){
    wake();
    readEEPROM();
}

float TempSensor::Read()
{
    Tamb = getTPAMB();
    float temp_amb = ((25 + 273.15f) + ((float)Tamb - (float)PTAT_25) * (1 / (float)M));

    Tobj = getTPOBJ();
    float temp0 = powf(temp_amb, 3.8f);
    float temp1 = (((float) Tobj) - ((float)U0)) / k;
    float temp_object = powf((temp0 + temp1), 0.2631578947f);

    return temp_object;
}
