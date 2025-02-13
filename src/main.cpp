#include <Arduino.h>
#include <stm32yyxx_ll_tim.h>
#include <quadratureDecoder.h>

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

// Revolutions per second of the encoder shaft
volatile long revPerSec = 0;

// Previously stored Tick Count, initialized to zero upon reset
volatile uint32_t prevTickCnt = 0;

// Function signature of speed calculation interrupt
void calculateRevPerSec();

// Create an QuadratureDecoder object, this wraps around the functionality of the decoder
// The Timer used MUST support Quadrature decoding, otherwise the functions will not work
QuadratureDecoder quadDecoder(TIM1, T1CH1_Pin, T1CH2_Pin);

// To obtain an accurate speed measurement I will obtain the rate of change
// of ticks per second, and divide that by the number of ticks in a full rotation
// This is a simple approach that gives an input frequency range of 0 to ~65 Khz.
HardwareTimer *SpeedTimer = new HardwareTimer(TIM2);

// Define speed calculation interrupt
void calculateRevPerSec(){

  uint32_t currentTickCount = quadDecoder.getCountQuad(TICK_FORMAT);
  // Calculate the ticks per second
  long tickDiff = currentTickCount-prevTickCnt;
  revPerSec = tickDiff; 

  prevTickCnt = currentTickCount;  
}

void setup()
{
  Serial.begin(9600);

  // Setup the speed measurement timer
  // SpeedTimer->setMode(ALL_CHANNELS, TIMER_OUTPUT_COMPARE_PWM1, pin);
  SpeedTimer->setOverflow(1,HERTZ_FORMAT);
  SpeedTimer->attachInterrupt(calculateRevPerSec);

  // Start speed timer
  SpeedTimer->resume();

}

void loop()
{
  Serial.println(quadDecoder.getCountQuad(TICK_FORMAT));
  Serial.println(revPerSec);
  Serial.println("~~~~");
  delay(1000);
}