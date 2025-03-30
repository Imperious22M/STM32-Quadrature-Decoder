// File to contain functions related to CAN

#include <Adafruit_MCP2515.h>
#include<SPI.h>

// SPI pins for CAN Module
#define MOSI_CAN PB5
#define MISO_CAN PB9
#define SCK_CAN  PB6
#define CS_CAN   PB3
// Set CAN bus baud rate
//#define CAN_BAUDRATE (500000)
#define CAN_BAUDRATE (250000)
// CAN packet transmission rate
#define CAN_UPDATE_RATE_HZ 100
// CAN BUS Adresses
#define frontLeftCANAddress 0x2e1
#define frontRightCANAddress 0x2e2
#define backLeftCANAddress 0x2e3
#define backRightCANAddress 0x2e4
// Global variables to use for CAN ISR
//int TPS1=0, TPS2=0, HZA=0, HZB=0;

// SPI Hardware Control
SPIClass SPI_2(MOSI_CAN,MISO_CAN,SCK_CAN);

// CAN Controller object
//Adafruit_MCP2515 canTransceiver(CS_CAN,&SPI_2);
//Adafruit_MCP2515 mcp(CS_CAN,MOSI_CAN,MISO_CAN,SCK_CAN);
Adafruit_MCP2515 canTransceiver(CS_CAN,MOSI_CAN,MISO_CAN,SCK_CAN);
// Timer for updating CAN at a set interval 
//HardwareTimer *canTimer = new HardwareTimer(TIM10);

// Start CAN communications
void setupCAN(){
  
  // Set SPI pins
  SPI_2.setDataMode(SPI_MODE0);


  // Set frequency to 8MHz
  // Amazon CAN modules have an 8MHz crystal by default
  //while(!Serial) delay(10);
  //delay(1000);
  pinMode(CS_CAN,OUTPUT);
  digitalWrite(CS_CAN,HIGH);
  //delay(2000);
  Serial.println("MCP2515 Sender test!");
  
  //SPI.setDataMode(SPI_MODE3);
  //SPI.begin();
  ////SPI.beginTransaction(SPISettings(8e6, MSBFIRST, SPI_MODE3));
  //SPI.transfer(0xFF);
  //SPI.end();
  //pinMode(SCK_CAN,OUTPUT);
  //pinMode(MISO_CAN,INPUT_PULLDOWN);
  //pinMode(MOSI_CAN,INPUT_PULLDOWN);
  
  canTransceiver.setClockFrequency(8e6);
   if (!canTransceiver.begin(CAN_BAUDRATE)) {
     pinMode(LED_BUILTIN,OUTPUT);
     while(1) {
        digitalWrite(LED_BUILTIN,HIGH);
        delay(100);
        digitalWrite(LED_BUILTIN,LOW);
        Serial.println("Error initializing MCP2515.");
        delay(100);
     if(canTransceiver.begin(CAN_BAUDRATE)){
      break;
     }
     }
   }

}
