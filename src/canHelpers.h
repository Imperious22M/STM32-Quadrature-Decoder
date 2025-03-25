// File to contain functions related to CAN

#include <Adafruit_MCP2515.h>
#include <SPI.h>

// SPI pins for CAN Module
#define MOSI_CAN PB15 
#define MISO_CAN PB14
#define SCK_CAN  PB13
#define CS_CAN   PB12
// Set CAN bus baud rate
#define CAN_BAUDRATE (500000)
// CAN packet transmission rate
#define CAN_UPDATE_RATE_HZ 100
// CAN BUS Adresses
#define frontLeftCANAddress 0x2e1
#define frontRightCANAddress 0x2e2
#define backLeftCANAddress 0x2e3
#define backRightCANAddress 0x2e4
// Global variables to use for CAN ISR
//int TPS1=0, TPS2=0, HZA=0, HZB=0;

// CAN Controller object
Adafruit_MCP2515 canTransceiver(CS_CAN,MOSI_CAN,MISO_CAN,SCK_CAN);
// Timer for updating CAN at a set interval 
HardwareTimer *canTimer = new HardwareTimer(TIM9);

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
