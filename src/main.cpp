#include <Arduino.h>
#include <stm32yyxx_ll_tim.h>
#include <quadratureDecoder.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_MCP2515.h>


/*
  Quadrature Decoder example using hardware timer 1 (Ch1 and Ch2)
  4x mode, with hardware filtering and edge detection
*/

#if !defined(STM32_CORE_VERSION) || (STM32_CORE_VERSION  < 0x01090000)
#error "Due to API change, this sketch is compatible with STM32_CORE_VERSION  >= 0x01090000"
#endif

#define T1CH1_Pin PA8 // Quadrature A channel input pin
#define T1CH2_Pin PA9 // Quadrature B channel input pin
#define ticksPerRevolution 50*4 // Number of "ticks" per revolution
#define teethNum 25 // number of teeth the gear has

// SPI pins for CAN Module
#define MOSI_CAN PA7 
#define MISO_CAN PA6
#define SCK_CAN  PA5
#define CS_CAN   PA4
#define CS_PIN    PA4
// Set CAN bus baud rate
#define CAN_BAUDRATE (250000)

#define SPEED_UPDATE_RATE_HZ 50 // The rate at which speed is updated in Hz
#define CAN_UPDATE_RATE_HZ 10 // The rate at which CAN messages are transmited in Hz

// Function signature of speed calculation interrupt
void calcTickPerSec();

// Function signature to send speed and direction to CAN bus
void sendCANMessage();

// Create an QuadratureDecoder object, this wraps around the functionality of the decoder
// The Timer used MUST support Quadrature decoding, otherwise the functions will not work
QuadratureDecoder quadDecoder(TIM1, T1CH1_Pin, T1CH2_Pin);

// Hardware timer to send information to the CAN bus at a rate of 100 Hz
HardwareTimer *CANTimer = new HardwareTimer(TIM4);

// Timer Timer
HardwareTimer *SpeedTimer = new HardwareTimer(TIM3);

// CAN Controller object
Adafruit_MCP2515 mcp(CS_CAN,MOSI_CAN,MISO_CAN,SCK_CAN);

// Send a CAN message with speed and direction
void sendCanMessage(){
  mcp.beginPacket(0x12);
  mcp.write('a');
  mcp.write('a');
  mcp.write('a');
  mcp.write('a');
  mcp.write('a');
  mcp.endPacket();
}


volatile uint32_t overflowTime = 0; // 
// Revolutions per second of the encoder shaft
volatile float ticksPerSec = 0;
// Previously stored Tick Count, initialized to zero upon reset
volatile uint16_t prevTickCnt = 0;
void calcTickPerSec()
{
  uint32_t curTickCnt = quadDecoder.getCountQuad(TICK_FORMAT);
  if(prevTickCnt == curTickCnt){
    overflowTime++;
    ticksPerSec = 0;
    //Serial.println("Overflow");
    return;
  }else {
    // Calculate the number of ticks per second
    double timeExtra = 0;

    // Calculate the number of time that has passed since the last tick
    if(overflowTime > 0){
      timeExtra = (float)overflowTime/SPEED_UPDATE_RATE_HZ;
      overflowTime = 0;
    }

    ticksPerSec = (curTickCnt - prevTickCnt) / (1.0/SPEED_UPDATE_RATE_HZ + timeExtra);
    prevTickCnt = curTickCnt;
    //Serial.println(ticksPerSec);
  }

}

void setup()
{
  Serial.begin(9600);

  // Setup CAN update Timer
  CANTimer->setOverflow(CAN_UPDATE_RATE_HZ,HERTZ_FORMAT); // Send CAN messages at the set Hz rate
  CANTimer->attachInterrupt(sendCanMessage);
  //Start CAN timer
  //CANTimer->resume();

  // Start speed timer
  SpeedTimer->setOverflow(SPEED_UPDATE_RATE_HZ,HERTZ_FORMAT); // Update speed at the set Hz rate
  SpeedTimer->attachInterrupt(calcTickPerSec); // Attach the speed calculation function to the timer
  SpeedTimer->resume();

  pinMode(CS_PIN,OUTPUT);

  // The amazon CAN modules have 8 Mhz crystals by default
  mcp.setClockFrequency(8e6);

  // If CAN controller is not found, fail to read encoders
  if (!mcp.begin(CAN_BAUDRATE)) {
    while(1) {
    delay(1000);
    Serial.println("Error initializing MCP2515.");
    }
  }

}

void loop()
{
  Serial.print("Encoder Count: ");
  Serial.println(quadDecoder.getCountQuad(TICK_FORMAT));
  Serial.print("Direction: ");
  Serial.println(quadDecoder.getDirBit()? "Backward":"Forward");
  Serial.println(quadDecoder.getDirBit());
  Serial.print("Ticks per second: ");
  Serial.println(ticksPerSec);
  Serial.println("~~~~");
  delay(1000);
}