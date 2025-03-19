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
#define T2CH1_Pin PA15 // Quadrature A channel input pin
#define T2CH2_Pin PB3 // Quadrature B channel input pin
#define ticksPerRevolution 50*4 // Number of "ticks" per revolution
#define teethNum 25 // number of teeth the gear has

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

// Speed update rate in HZ
#define SPEED_UPDATE_RATE_HZ 100 
// Overflow timeout rate, must be LOWER than SPEED_UPDATE_RATE
#define OVERFLOW_TIMEOUT_HZ 1
// Period of update rate
const float speedTimeRate =  1.0/SPEED_UPDATE_RATE_HZ; 
// Approximate overflowTime counter value for 
// trigerring timeout. At this value the overflow 
// value is triggered and ticks/s is set to 0
// Approximated to the truncated decimal value
const int overflowTimeoutCnt = (SPEED_UPDATE_RATE_HZ/OVERFLOW_TIMEOUT_HZ);
// Minimum HZ value of OVERFLOW_TIMEOUT_HZ, tied to a max possible 16 bit counter
const float minimumOverflowHz = ((float)SPEED_UPDATE_RATE_HZ/INT16_MAX);
// Revolutions per second of the encoder shaft #1
volatile float ticksPerSec1 = 0;
// Revolutions per second of the encoder shaft #2
volatile float ticksPerSec2 = 0;
// previous tick variables
volatile int prevTickCnt1 = 0;
volatile int prevTickCnt2 = 0;
// Tick Differential variables
volatile int tickDiff1 = 0;
volatile int tickDiff2 = 0;

// Function signature of speed calculation interrupt
void calcTickPerSec();
// Function signature to send speed and direction to CAN bus
void sendCANMessage();

// Create a QuadratureDecoder object, this wraps around the functionality of the decoder
// The Timer used MUST support Quadrature decoding, otherwise the functions will not work
QuadratureDecoder quadDecoder(TIM1, T1CH1_Pin, T1CH2_Pin);
// Second quadrature decoder
QuadratureDecoder quadDecoder2(TIM4, PB6, PB7);
// Hardware timer to send information to the CAN bus at a rate of 100 Hz
HardwareTimer *CANTimer = new HardwareTimer(TIMER_TONE);
// Timer Timer
HardwareTimer *SpeedTimer = new HardwareTimer(TIMER_SERVO);
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


// Faster tick diff function callback
void calcTickDiff(){
  // Current count of each shaft
  uint32_t curTickCnt1 = quadDecoder.getCountQuad(TICK_FORMAT);
  uint32_t curTickCnt2 = quadDecoder2.getCountQuad(TICK_FORMAT);
  tickDiff1 = curTickCnt1 - prevTickCnt1;
  tickDiff2 = curTickCnt2 - prevTickCnt2;
}


// Calculates ticks/second at  a rate of SPEED_UPDATE_HZ
// Use direction as discriminant and account for over/underflow
void calcTickPerSec_1()
{
  // Previously stored Tick Count, initialized to zero upon reset
  static volatile uint16_t prevTickCnt1 = 0;
  static volatile uint16_t prevTickCnt2 = 0;
  // Overflow time for each channel
  static volatile int overflowTime1 = 0;
  static volatile int overflowTime2 = 0;
  // Current count of each shaft
  uint32_t curTickCnt1 = quadDecoder.getCountQuad(TICK_FORMAT);
  uint32_t curTickCnt2 = quadDecoder2.getCountQuad(TICK_FORMAT);
  // Direction for shafts
  static int prevDirection1 = quadDecoder.getDirBit();
  static int prevDirection2 = quadDecoder2.getDirBit();
  // Debug vars
  static long prevMillis1 = millis();


  // Increase overflow time
  if(prevTickCnt1 == curTickCnt1){
    overflowTime1++;
    // Reset overflow time if an increase no change has been detected 
    // in the set number of overflows
    if(overflowTime1==overflowTimeoutCnt){
      overflowTime1 = 0;
      ticksPerSec1 = 0;
    }
  }
  if(prevTickCnt2 == curTickCnt2){
    overflowTime2++;
    if(overflowTime2==overflowTimeoutCnt){
      overflowTime2 = 0;
      ticksPerSec2 = 0;
    }
  }

  // Calculate the ticks/second assuming ticks/speedTimeRate remain constant
  if(prevTickCnt1 == curTickCnt1 && prevTickCnt2 == curTickCnt2){
    // If no tick change, return.
    return;
  }else{
    // Calculate the number of ticks per second
    double timeExtra1 = 0;
    double timeExtra2 = 0;
    // Temporary direction variable to account for overflow
    bool direction = 0;  

    // Calculate the number of time that has passed since the last tick
    if(overflowTime1 > 0){
      timeExtra1 = (float)overflowTime1*speedTimeRate;
      overflowTime1 = 0;
    }
    if(overflowTime2 > 0){
      timeExtra2 = (float)overflowTime2*speedTimeRate;
      overflowTime2 = 0;
    }
    // TODO ADD OVERFLOW TIME

    // Calculate Ticks/Second based on the difference of tick CNT
    if(curTickCnt1>prevTickCnt1){
      ticksPerSec1 = (curTickCnt1 - prevTickCnt1) / (speedTimeRate);
      direction = 0;
    } else { 
    // Account for overflow
      ticksPerSec1 = (prevTickCnt1 - curTickCnt1) / (speedTimeRate);
      direction = 1;
    }

   // If we have overflowed, assume the quadrature counter has also overflowed
   // and correct the count to account for this
    if(ticksPerSec1>UINT16_MAX){
      if(direction == 0 && prevDirection1 == 1){
        // Overflowed from 0 -> 65535
        ticksPerSec1 = (prevTickCnt1 + (UINT16_MAX - curTickCnt1)) / (speedTimeRate);
      }else{
        ticksPerSec1 = (curTickCnt1 + (UINT16_MAX - prevTickCnt1)) / (speedTimeRate);
      }
     }
    // Store previous direction for use when the counter overflows
    prevDirection1 = direction;

    direction = 0; // reset direction for second encoder
    // Do the same Ticks Per Second calculation for the second encoder
    if(curTickCnt2>prevTickCnt2){
      ticksPerSec2 = (curTickCnt2 - prevTickCnt2) / (speedTimeRate);
      direction = 0;
    } else { 
    // Account for overflow
      ticksPerSec2 = (prevTickCnt2 - curTickCnt2) / (speedTimeRate);
      direction = 1;
    }
    if(ticksPerSec2>UINT16_MAX){
      if(direction == 0 && prevDirection2 == 1){
        // Overflowed from 0 -> 65535
        ticksPerSec2 = (prevTickCnt2 + (UINT16_MAX - curTickCnt2)) / (speedTimeRate);
      }else{
        ticksPerSec2 = (curTickCnt2 + (UINT16_MAX - prevTickCnt2)) / (speedTimeRate);
      }
     }
    // Store previous direction for use when the counter overflows
    prevDirection2 = direction;

    // Error message if overflow remains after it was accounted for
    if(ticksPerSec1>UINT16_MAX || ticksPerSec2>UINT16_MAX){
      if(ticksPerSec1>UINT16_MAX){
        Serial.println("Overflow in TPS1:1000");
      }
      if(ticksPerSec2>UINT16_MAX){
        Serial.println("Overflow in TPS2:2000");
      }
      Serial.println("Overflow error! Ticks per second is too high!");
      Serial.print("Ticks per second: ");
      Serial.println(ticksPerSec1);  
      Serial.print("Current tick count: ");
      Serial.println(curTickCnt1);
      Serial.print("Previous tick count: ");
      Serial.println(prevTickCnt1);
      Serial.print("Direction: ");
      Serial.println(direction);
      Serial.println("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
      Serial.println();
      return;
    }

    
    if(ticksPerSec1>=200){
      Serial.println("200 error!!");
      Serial.print("Ticks per second: ");
      Serial.println(ticksPerSec1);  
      Serial.print("Current tick count: ");
      Serial.println(curTickCnt1);
      Serial.print("Previous tick count: ");
      Serial.println(prevTickCnt1);
      Serial.print("Direction: ");
      Serial.println(direction);
      Serial.print("Overflow Time:");
      Serial.println(overflowTime1);
      Serial.print("Millis Diff: ");
      Serial.println(millis()-prevMillis1);
      Serial.println("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
      Serial.println();
    }

    // Store the current tick count for the next iteration
    prevTickCnt1 = curTickCnt1;
    prevTickCnt2 = curTickCnt2;
    prevMillis1 = millis();
  }
}

void setup()
{
  Serial.begin(9600);


    pinMode(PB6, INPUT);
    pinMode(PB7, INPUT);
    //pinMode(T2CH1_Pin, INPUT);
    //pinMode(T2CH2_Pin, INPUT); 
      // configure TIM2 as Encoder input
    //RCC->APB1ENR |= 0x00000001;  // Enable clock for TIM2
 
    //TIM2->CR1   = 0x0001;     // CEN(Counter ENable)='1'     < TIM control register 1
    //TIM2->SMCR  = 0x0003;     // SMS='011' (Encoder mode 3)  < TIM slave mode control register
    //TIM2->CCMR1 = 0x0101;     // CC1S='01' CC2S='01'         < TIM capture/compare mode register 1
    //TIM2->CCMR2 = 0x0000;     //                             < TIM capture/compare mode register 2
    //TIM2->CCER  = 0x0011;     // CC1P CC2P                   < TIM capture/compare enable register
    //TIM2->PSC   = 0x0000;     // Prescaler = (0+1)           < TIM prescaler
    //TIM2->ARR   = 0xffffffff; // reload at 0xfffffff         < TIM auto-reload register
    ////Tim2->BDTR
    //TIM2->CNT = 0x0000;  //reset the counter before we use it  

  // Setup CAN update Timer
  CANTimer->setOverflow(CAN_UPDATE_RATE_HZ,HERTZ_FORMAT); // Send CAN messages at the set Hz rate
  CANTimer->attachInterrupt(sendCanMessage);
  //Start CAN timer
  //CANTimer->resume();

  // Check overflow timeout limits
  if(OVERFLOW_TIMEOUT_HZ>=SPEED_UPDATE_RATE_HZ || OVERFLOW_TIMEOUT_HZ<minimumOverflowHz){
    Serial.println("Incorrect overflow timeout value! Must be below than update rate and above minimum overflow.");
    delay(1000);
  }

  // Start speed timer
  SpeedTimer->setOverflow(SPEED_UPDATE_RATE_HZ,HERTZ_FORMAT); // Update speed at the set Hz rate
  //SpeedTimer->attachInterrupt(calcTickPerSec_1); // Attach the speed calculation function to the timer
  SpeedTimer->attachInterrupt(calcTickDiff); // Attach the speed calculation function to the timer
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
  //Serial.print("Count_1:");
  //Serial.println(quadDecoder.getCountQuad(TICK_FORMAT));
  Serial.print("Dir1:");
  Serial.println(quadDecoder.getDirBit());
  Serial.print("TPS1:");
  Serial.println(ticksPerSec1);
  //Serial.print("Count_2:");
  //Serial.println(quadDecoder2.getCountQuad(TICK_FORMAT));
  Serial.print("TPS2:");
  Serial.println(ticksPerSec2);
  delay(100);
}