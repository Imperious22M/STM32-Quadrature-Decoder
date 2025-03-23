// File to contain functions related to CAN

#include <Adafruit_MCP2515.h>

// SPI pins for CAN Module
#define MOSI_CAN PA7 
#define MISO_CAN PA6
#define SCK_CAN  PA5
#define CS_CAN   PA4
#define CS_PIN    PA4
// Set CAN bus baud rate
#define CAN_BAUDRATE (500000)
// CAN packet transmission rate
#define CAN_UPDATE_RATE_HZ 100
  // Setup CAN update Timer
  //CANTimer->setOverflow(1,HERTZ_FORMAT); // Send CAN messages at the set Hz rate
  //CANTimer->setOverflow(62499);
  //CANTimer->setPrescaleFactor(1151);
  //CANTimer->attachInterrupt(calc1Sec);
  //Start CAN timer
  //CANTimer->resume();

  // CAN Bus module configuration
  //pinMode(CS_PIN,OUTPUT);

  // The amazon CAN modules have 8 Mhz crystals by default
  //canTransceiver.setClockFrequency(8e6);
  // If CAN controller is not found, fail to read encoders
 // if (!canTransceiver.begin(CAN_BAUDRATE)) {
 //   while(1) {
 //   delay(1000);
 //   Serial.println("Error initializing canTransceiver2515.");
 //   }
 // }

// CAN Controller object
Adafruit_MCP2515 canTransceiver(CS_CAN,MOSI_CAN,MISO_CAN,SCK_CAN);
// Send a CAN message with speed and direction
void sendCanMessage(){
  canTransceiver.beginPacket(0x12);
  canTransceiver.write('a');
  canTransceiver.write('a');
  canTransceiver.write('a');
  canTransceiver.write('a');
  canTransceiver.write('a');
  canTransceiver.endPacket();
}