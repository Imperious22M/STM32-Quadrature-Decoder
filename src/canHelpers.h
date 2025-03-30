// File to contain functions related to CAN

//#include <Adafruit_MCP2515.h>
#include <mcp_can.h>
#include<SPI.h>

// SPI pins for CAN Module
#define MOSI_CAN PB15
#define MISO_CAN PB14
#define SCK_CAN  PB13
#define CS_CAN   PB12
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

//Adafruit_MCP2515 canTransceiver(CS_CAN,MOSI_CAN,MISO_CAN,SCK_CAN);

MCP_CAN CAN0(&SPI_2,CS_CAN);     // Set CS to pin 10
// Start CAN communications

void setupCAN(){
  
  // Set SPI pins
  //SPI_2.setDataMode(SPI_MODE0);


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
  
  //canTransceiver.setClockFrequency(8e6)  // Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s and the masks and filters disabled.
  if(CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) Serial.println("MCP2515 Initialized Successfully!");
  else
  {
    pinMode(LED_BUILTIN,OUTPUT);
    while(1){

        digitalWrite(LED_BUILTIN,HIGH);
        delay(100);
        digitalWrite(LED_BUILTIN,LOW);
        Serial.println("Error initializing MCP2515.");
        delay(100);
    Serial.println("MCP2515 Initialization Failed!");
    }
  }


  // if (!CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
  //   pinMode(LED_BUILTIN,OUTPUT);
  //   while(1) {
  //      digitalWrite(LED_BUILTIN,HIGH);
  //      delay(100);
  //      digitalWrite(LED_BUILTIN,LOW);
  //      Serial.println("Error initializing MCP2515.");
  //      delay(100);
  //   if(CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ) == CAN_OK){
  //    break;
  //   }
  //   }
  // }

   //CAN0.mcp2515_setCANCTRL_Mode(MODE_ONESHOT);

   CAN0.setMode(MCP_NORMAL);   // Change to normal mode to allow messages to be transmitted
}
