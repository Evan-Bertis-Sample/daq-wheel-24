#include <Arduino.h>
#include <Wire.h>
#include "speedSensor.h"
#include "tempSensor.h"

int hallSensorPin = 36;

SpeedSensor speedSensor(hallSensorPin);

TempSensor* tempSensor = nullptr;

void i2cScan()
{
    // scan for i2c devices
    byte error, address;
    int nDevices;

    Serial.println("Scanning...");

    nDevices = 0;
    for (address = 1; address < 127; address++)
    {
        // The i2c_scanner uses the return value of
        // the Write.endTransmission to see if
        // a device did acknowledge to the address.
        Wire.beginTransmission(address);
        error = Wire.endTransmission();
        if (error == 0)
        {
            Serial.print("I2C device found at address 0x");
            if (address < 16)
                Serial.print("0");
            Serial.print(address, HEX);
            Serial.println(" !");

            nDevices++;
        }
        else if (error == 4)
        {
            Serial.print("Unknown error at address 0x");
            if (address < 16)
                Serial.print("0");
            Serial.println(address, HEX);
        }
    }
    if (nDevices == 0)
        Serial.println("No I2C devices found\n");
    else
        Serial.println("done\n");
}
// Workaround
void IRAM_ATTR handleInterrupt()
{
    speedSensor.Interrupt();
}

void setup()
{
    Serial.begin(115200);
    Wire.begin();
    i2cScan();
    tempSensor=new TempSensor();
    
    attachInterrupt(digitalPinToInterrupt(hallSensorPin), handleInterrupt, FALLING);
}

void loop()
{
    float objectTemp = (*tempSensor).Read();
    float RPS = speedSensor.Read();
    Serial.print("Object Temp: ");
    Serial.println(objectTemp);
    Serial.print("RPS: ");
    Serial.println(RPS);
    delay(1000);
}
