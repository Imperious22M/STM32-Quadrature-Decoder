// File to contain functions related to CAN

#include <Adafruit_MCP2515.h>
#include <SPI.h>

// SPI pins for CAN Module
#define MOSI_CAN PB15 
#define MISO_CAN PB14
#define SCK_CAN  PB13
#define CS_CAN   PB12
// Set CAN bus baud rate
#define CAN_BAUDRATE (250000)
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


// CAN Controller object
Adafruit_MCP2515 canTransceiver(CS_CAN,MOSI_CAN,MISO_CAN,SCK_CAN);

// Start CAN communications
void setupCAN(){

  pinMode(CS_CAN,OUTPUT);
  SPI.setMISO(MISO_CAN);
  SPI.setMOSI(MOSI_CAN);
  SPI.setSCLK(SCK_CAN);
  SPI.setSSEL(CS_CAN);
  SPI.begin();

  //pinMode(SCK_CAN,OUTPUT);
 // The amazon CAN modules have 8 Mhz crystals by default
 canTransceiver.setClockFrequency(8e6);
 // If CAN controller is not found, fail to read encoders
if (!canTransceiver.begin(CAN_BAUDRATE)) {
   while(1) {
    if(canTransceiver.begin(CAN_BAUDRATE)){
      break;
    }
   delay(1000);
   Serial.println("Error initializing canTransceiver2515.");
   }
  }
}

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