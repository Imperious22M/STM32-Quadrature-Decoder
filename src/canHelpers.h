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

// SPI Hardware Control
SPIClass SPI_2(MOSI_CAN,MISO_CAN,SCK_CAN);

// MCP Controller instance
MCP_CAN CAN0(&SPI_2,CS_CAN);     // Set CS to pin 10

// Timer for CAN communications
HardwareTimer *canTimer;

// Setup CAN communiations
void setupCAN(){
  
  pinMode(CS_CAN,OUTPUT);
  digitalWrite(CS_CAN,HIGH);
  Serial.println("MCP2515 Sender test!");
  
  // Amazon CAN modules have an 8MHz crystal by default
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

  CAN0.setMode(MCP_NORMAL);   // Change to normal mode to allow messages to be transmitted
  CAN0.enOneShotTX();
}
