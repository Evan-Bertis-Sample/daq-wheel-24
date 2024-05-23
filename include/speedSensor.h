#include <Arduino.h>
#define NUM_GEAR_TEETH 72

class SpeedSensor
{
private:
    int sensorPin;
    int teethCount;
    unsigned long lastReadTime;
    float lastRead;

public:
    //
    // Create a FlowRateSensor object who's signal wire is at "sensorPin"
    //
    SpeedSensor(int sensorPin) : sensorPin(sensorPin), teethCount(0), lastReadTime(0), lastRead(0)
    {
        pinMode(sensorPin, INPUT_PULLUP);
    };

    //
    // Interrrupt to determine when gear tooth is detected
    //

    void Interrupt();

    //
    // Returns RPS
    //
    float Read();
};