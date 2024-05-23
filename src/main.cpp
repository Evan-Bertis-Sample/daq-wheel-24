#include <Arduino.h>
#include <Wire.h>
#include "speedSensor.h"
#include "tempSensor.h"
#include "esp_can.h"

ESPCAN can_bus{10, GPIO_NUM_32, GPIO_NUM_27};

VirtualTimerGroup timer_group{};

MakeSignedCANSignal(float, 0, 16, 0.01, 0) temp_tx_signal_1{};
MakeSignedCANSignal(float, 16, 16, 0.01, 0) temp_tx_signal_2{};
MakeSignedCANSignal(float, 32, 16, 0.01, 0) temp_tx_signal_3{};
MakeSignedCANSignal(float, 48, 16, 0.01, 0) temp_tx_signal_4{};

MakeSignedCANSignal(float, 0, 16, 0.01, 0) speed_tx_signal_1{};
MakeSignedCANSignal(float, 16, 16, 0.01, 0) speed_tx_signal_2{};


//Board #1
CANTXMessage<4>
    temp_tx_message{can_bus, 0x410, 4, 100, timer_group, temp_tx_signal_1, temp_tx_signal_2, temp_tx_signal_3, temp_tx_signal_4 };


CANTXMessage<4>
    speed_tx_message{can_bus, 0x411, 4, 100, timer_group, speed_tx_signal_1, speed_tx_signal_2 };

// Board #2
// CANTXMessage<4>
//     temp_tx_message{can_bus, 0x412, 4, 100, timer_group, temp_tx_signal_1, temp_tx_signal_2, temp_tx_signal_3, temp_tx_signal_4 };

// ONLY PUT SPEED FOR BOARD 1


bool DEBUG_MODE = true;

// Sensor DEFS
int hallSensor1Pin = 23;
int hallSensor2Pin = 0; //CHANGE
SpeedSensor speedSensor1(hallSensor1Pin);
SpeedSensor speedSensor2(hallSensor2Pin);

TempSensor *tempSensor1, *tempSensor2, *tempSensor3, *tempSensor4 = nullptr;


void i2cScan()
{
    // scan for i2c devices
    byte error, address;
    int nDevices;

    Serial.println("Scanning...");

    nDevices = 0;
    for (address = 0; address < 255; address++)
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
    if(DEBUG_MODE){
        Serial.println("Gear tooth 1!");
    }
    speedSensor1.Interrupt();
}
void IRAM_ATTR handleInterrupt2()
{
    if(DEBUG_MODE){
        Serial.println("Gear tooth 2!");
    }
    speedSensor2.Interrupt();
}

void one_sec_task()
{
    float wheelTemp1 = tempSensor1->Read();
    float wheelTemp2 = tempSensor2->Read();
    float wheelTemp3 = tempSensor3->Read();
    float wheelTemp4 = tempSensor4->Read();
    float RPS1 = speedSensor1.Read();
    float RPS2 = speedSensor2.Read();

    temp_tx_signal_1 = wheelTemp1;
    temp_tx_signal_2 = wheelTemp2;
    temp_tx_signal_3 = wheelTemp3;
    temp_tx_signal_4 = wheelTemp4;

    
    speed_tx_signal_1 = RPS1;
    speed_tx_signal_2 = RPS2;

    if (DEBUG_MODE)
    {
        Serial.print("Wheel Temp 1: ");
        Serial.println(wheelTemp1);
        Serial.print("Wheel Temp 2: ");
        Serial.println(wheelTemp2);
        Serial.print("Wheel Temp 3: ");
        Serial.println(wheelTemp3);
        Serial.print("Wheel Temp 4: ");
        Serial.println(wheelTemp4);

        Serial.print("RPS 1: ");
        Serial.println(RPS1);
        Serial.print("RPS 2: ");
        Serial.println(RPS2);
    }

    can_bus.Tick();
}

void setup()
{

    Serial.begin(115200);

    Wire1.begin(19,18);
    Wire.begin(25,26);


    i2cScan();

    tempSensor1 = new TempSensor(0x0C, Wire);
    tempSensor2 = new TempSensor(0x0E, Wire);
    tempSensor3 = new TempSensor(0x0C, Wire1);
    tempSensor4 = new TempSensor(0x0E, Wire1);

    can_bus.Initialize(ICAN::BaudRate::kBaud1M);
    timer_group.AddTimer(1000, one_sec_task);

    attachInterrupt(digitalPinToInterrupt(hallSensor1Pin), handleInterrupt, FALLING);
    attachInterrupt(digitalPinToInterrupt(hallSensor2Pin), handleInterrupt2, FALLING);
}

void loop(){
    timer_group.Tick(millis());
}
