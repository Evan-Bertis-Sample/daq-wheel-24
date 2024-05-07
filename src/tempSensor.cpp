#include <Arduino.h>
#include <tempSensor.h>




void TempSensor::readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t *dest)
{
    I2C->beginTransmission(address); // Initialize the Tx buffer
    I2C->write(subAddress);          // Put slave register address in Tx buffer
    I2C->endTransmission(false);     // Send the Tx buffer, but send a restart to keep connection alive
    uint8_t i = 0;
    I2C->requestFrom(address, count); // Read bytes from slave register address
    while (I2C->available())
    {
        dest[i++] = I2C->read();
    } // Put read results in the Rx buffer
}
void TempSensor::writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
    I2C->beginTransmission(address); // Initialize the Tx buffer
    I2C->write(subAddress);          // Put slave register address in Tx buffer
    I2C->write(data);                // Put data in Tx buffer
    I2C->endTransmission();          // Send the Tx buffer
}
void TempSensor::readEEPROM()
{
    uint8_t rawData[2] = {0, 0};
    writeByte(slave_address, CALIPILE_EEPROM_CONTROL, 0x80);
    // Read calibration constants
    readBytes(slave_address, CALIPILE_EEPROM_PTAT25, 2, &rawData[0]);
    PTAT_25 = ((uint16_t)rawData[0] << 8) | rawData[1];

    readBytes(slave_address, CALIPILE_EEPROM_M, 2, &rawData[0]);
    M = ((uint16_t)rawData[0] << 8) | rawData[1];
    M /= 100;

    readBytes(slave_address, CALIPILE_EEPROM_U0, 2, &rawData[0]);
    U0 = ((uint16_t)rawData[0] << 8) | rawData[1];
    U0 += 32768;

    readBytes(slave_address, CALIPILE_EEPROM_UOUT1, 2, &rawData[0]);
    uint32_t Uout1 = ((uint16_t)rawData[0] << 8) | rawData[1];
    Uout1 *= 2;

    uint8_t Tobj1;
    readBytes(slave_address, CALIPILE_EEPROM_TOBJ1, 1, &Tobj1);

    writeByte(slave_address, CALIPILE_EEPROM_CONTROL, 0x00);

    k = ((float)(Uout1 - U0)) / (powf((float)(Tobj1 + 273.15f), 3.8f) - powf(25.0f + 273.15f, 3.8f));
}
void TempSensor::wake()
{
    writeByte(0x00, 0x04, 0x00); // issue general call and reload command
    delay(1);
}

uint16_t TempSensor::getTPAMB()
{
    uint8_t rawData[2] = {0, 0};
    readBytes(slave_address, CALIPILE_TPAMBIENT, 2, &rawData[0]);
    uint16_t temp = ((uint16_t)(rawData[0] & 0x7F) << 8) | rawData[1];
    return temp;
}
uint32_t TempSensor::getTPOBJ()
{
    uint8_t rawData[3] = {0, 0, 0};
    readBytes(slave_address, CALIPILE_TPOBJECT, 3, &rawData[0]);
    uint32_t temp = ((uint32_t)((uint32_t)rawData[0] << 24) | ((uint32_t)rawData[1] << 16) | ((uint32_t)rawData[2] & 0x80) << 8) >> 15;
    return temp;
}

TempSensor::TempSensor(uint8_t address, TwoWire &bus){
    I2C = &bus;
    slave_address = address;
    // wake();
    // readEEPROM();
}

float TempSensor::Read()
{
    return 1.00;

    uint16_t Tamb = getTPAMB();
    float temp_amb = (25 + ((float)Tamb - (float)PTAT_25) * (1 / (float)M));

    uint32_t Tobj = getTPOBJ();
    float temp0 = powf(temp_amb, 3.8f);
    float temp1 = (((float) Tobj) - ((float)U0)) / k;
    float temp_object = powf((temp0 + temp1), 0.2631578947f);

    return temp_object;
}
