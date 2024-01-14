#include "speedSensor.h"
#include <Arduino.h>

void IRAM_ATTR SpeedSensor::Interrupt(){
    this->teethCount++;

}
float SpeedSensor::Read(){
    
    unsigned long readTime = millis();
    float timeSinceLastRead = (float)(readTime - this->lastReadTime) / 1000;
    this->lastReadTime = readTime;

    if (timeSinceLastRead < 10e-5)
        return this->lastRead;

    float RPS = this->teethCount / timeSinceLastRead / NUM_GEAR_TEETH;

    // Reset state
    this->lastRead = RPS;
    this->teethCount = 0;

    return RPS;
}