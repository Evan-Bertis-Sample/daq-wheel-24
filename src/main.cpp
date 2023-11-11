/* #include <Arduino.h>
void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
  pinMode(13, INPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
  int result = analogRead(13);
  Serial.println("----- WHEEL SENSOR -----");
  Serial.println(result);
  delay(1000);
  
} */
#include <Arduino.h>
#include <Wire.h>

//CaliPile Registers
#define CALIPILE_TPOBJECT            1
#define CALIPILE_TPAMBIENT           3

//EEPROM Registers
#define CALIPILE_EEPROM_CONTROL     31
#define CALIPILE_EEPROM_PTAT25      42
#define CALIPILE_EEPROM_M           44
#define CALIPILE_EEPROM_U0          46
#define CALIPILE_EEPROM_UOUT1       48
#define CALIPILE_EEPROM_TOBJ1       50

// Calipile Address
#define CALIPILE_ADDRESS 0x0C

// Collect Data
uint16_t Tamb;
uint32_t Tobj;

// Calibration Constants
uint32_t Uout1;
uint8_t  Tobj1;
uint16_t PTAT_25, M, U0;
float k;
/* 
// CAN SETUP
#pragma region CAN Setup
// Decide which can bus to use based on hardware
#if defined(ARDUINO_TEENSY40) || defined(ARDUINO_TEENSY41)
#include "teensy_can.h"
TeensyCAN<1> can_bus{};
#endif
#ifdef ARDUINO_ARCH_ESP32
#include "esp_can.h"
// TX pin and RX pin as input.
ESPCAN can_bus{};
#endif

// CAN addresses
const uint16_t pTrainAddr{0x420};
const uint16_t coolantAddr{0x421};

// Structure for handling timers
VirtualTimerGroup readTimer;

// * Setup for ambient_temp_signal
CANSignal<float, 32, 16, CANTemplateConvertFloat(0.1), CANTemplateConvertFloat(-40), false> ambientTempSignal{};
// * Setup for coolant_temp_signal
CANSignal<float, 16, 16, CANTemplateConvertFloat(0.1), CANTemplateConvertFloat(-40), false> coolant_temp_signal{};

// * Transmit pTrain signals
CANTXMessage<2> pTrainMessage{can_bus, pTrainAddr, 6, 100, readTimer, ambientTempSignal, coolant_temp_signal};

// * Setup for coolant_flow signal
CANSignal<float, 0, 16, CANTemplateConvertFloat(0.01), CANTemplateConvertFloat(0), false> coolantFlowSignal{};
// Transmit coolant signals
CANTXMessage<1> coolantMessage{can_bus, coolantAddr, 2, 500, readTimer, coolantFlowSignal};
#pragma endregion */

 

void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t* dest)
{  
  Wire.beginTransmission(address);   // Initialize the Tx buffer
  Wire.write(subAddress);            // Put slave register address in Tx buffer
  Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
  uint8_t i = 0;
  Wire.requestFrom(address, count);  // Read bytes from slave register address 
  while (Wire.available()) {dest[i++] = Wire.read(); } // Put read results in the Rx buffer
}
 
 void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
  Wire.beginTransmission(address);  // Initialize the Tx buffer
  Wire.write(subAddress);           // Put slave register address in Tx buffer
  Wire.write(data);                 // Put data in Tx buffer
  Wire.endTransmission();           // Send the Tx buffer
}
void readEEPROM()
{
  uint8_t rawData[2] = {0, 0};
  writeByte(CALIPILE_ADDRESS, CALIPILE_EEPROM_CONTROL, 0x80);
  //Read calibration constants
  readBytes(CALIPILE_ADDRESS, CALIPILE_EEPROM_PTAT25, 2, &rawData[0]);
  PTAT_25 = ( (uint16_t) rawData[0] << 8) | rawData[1];

  readBytes(CALIPILE_ADDRESS, CALIPILE_EEPROM_M, 2, &rawData[0]);
  M = ( (uint16_t) rawData[0] << 8) | rawData[1];
  M/=100;

  readBytes(CALIPILE_ADDRESS, CALIPILE_EEPROM_U0, 2, &rawData[0]);
  U0 = ( (uint16_t) rawData[0] << 8) | rawData[1];
  U0 += 32768;

  readBytes(CALIPILE_ADDRESS, CALIPILE_EEPROM_UOUT1, 2, &rawData[0]);
  Uout1 = ( (uint16_t) rawData[0] << 8) | rawData[1];
  Uout1 *= 2;

  readBytes(CALIPILE_ADDRESS, CALIPILE_EEPROM_TOBJ1, 1, &Tobj1);

  writeByte(CALIPILE_ADDRESS, CALIPILE_EEPROM_CONTROL, 0x00);

  k = ((float) (Uout1 - U0) )/(powf((float)(Tobj1 + 273.15f), 3.8f) - powf(25.0f + 273.15f, 3.8f) );
  

}
void i2cScan(){
  // scan for i2c devices
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ ) 
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmission to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
      
    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error==4) 
    {
      Serial.print("Unknown error at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
    
  }
void wake() {
  writeByte(0x00, 0x04, 0x00);  // issue general call and reload command
  delay(1);
}

uint16_t getTPAMB(){
  uint8_t rawData[2] = {0, 0};
  readBytes(CALIPILE_ADDRESS, CALIPILE_TPAMBIENT, 2, &rawData[0]);
  uint16_t temp = ( (uint16_t)(rawData[0] & 0x7F) << 8) | rawData[1] ; 
  return temp;
  }
uint32_t getTPOBJ(){
  uint8_t rawData[3] = {0, 0, 0};
  readBytes(CALIPILE_ADDRESS, CALIPILE_TPOBJECT, 3, &rawData[0]);
  uint32_t temp = ( (uint32_t) ( (uint32_t)rawData[0] << 24) | ( (uint32_t)rawData[1] << 16) | ( (uint32_t)rawData[2] & 0x80) << 8) >> 15; 
  return temp;
}
void setup() {
  Wire.begin();
  i2cScan();
  wake();
  Serial.begin(115200);
  readEEPROM();

  /* // Initialize CAN bus.
  can_bus.Initialize(ICAN::BaudRate::kBaud1M); */
}
void loop() {
  Tamb = getTPAMB();
  float temp_amb = ((25 + 273.15f) + ((float)Tamb - (float)PTAT_25) * (1/(float)M));
  Serial.print("Temperature Ambient: ");
  Serial.println(temp_amb);

  Tobj = getTPOBJ();
  float temp0 = powf(temp_amb, 3.8f);
  float temp1 = ( ((float) Tobj) - ((float) U0)  ) / k ;
  float temp_object = powf( (temp0 + temp1), 0.2631578947f );
  Serial.print("Temperature Object: ");
  Serial.println(temp_object);

  delay(1000);
}
//Interrupt?