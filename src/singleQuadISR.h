
#include <Arduino.h>
// Single decoder pinout and settings
// Timers used: TIM1 and TIM 4
// TIM1 -> Single Encoder A
// TIM4 -> Single Encoder B
// QuadA: PA10
// QuadBL PB8
#define SINGLE_ENCODER_A_PIN PA10 // Single encoder A
#define ENCODER_A_CHANNEL 3
#define SINGLE_ENCODER_B_PIN PB8 // Single encoder B
#define ENCODER_B_CHANNEL 3
#define SINGLE_ENCODER_REFRESH_RATE 1 // Refresh rate in HZ (Register recycle time)
volatile uint32_t frequencyMeasuredA, lastCaptureA = -1, currentCaptureA;
volatile uint32_t frequencyMeasuredB, lastCaptureB = -1, currentCaptureB;
uint32_t inputFreqA = 0;
uint32_t inputFreqB = 0;
volatile uint32_t rolloverComparecountA = 0;
volatile uint32_t rolloverComparecountB = 0;
HardwareTimer *singleDecTimerA;
HardwareTimer *singleDecTimerB;

// Single encoder decoding functions~~~~~~~~~~~~

// Frequency measure of the single encoderAChannel decoder
void inputCaptureRisingA(void)
{
  currentCaptureA = singleDecTimerA->getCaptureCompare(ENCODER_A_CHANNEL);
  /* frequency computation */
  if (currentCaptureA > lastCaptureA) {
    frequencyMeasuredA = inputFreqA / (currentCaptureA - lastCaptureA);
  }
  else if (currentCaptureA <= lastCaptureA) {
    /* 0x1000 is max overflow value */
    frequencyMeasuredA = inputFreqA / (0x10000 + currentCaptureA - lastCaptureA);
  }
  lastCaptureA = currentCaptureA;
  rolloverComparecountA = 0;
}

/* In case of timer rollover, frequency is to low to be measured set value to 0
   To reduce minimum frequency, it is possible to increase prescaler. But this is at a cost of precision. */
void rolloverCaptureRisingA(void)
{
  rolloverComparecountA++;

  if (rolloverComparecountA > 1)
  {
    frequencyMeasuredA = 0;
  }
}

void inputCaptureRisingB(void)
{
  currentCaptureB = singleDecTimerB->getCaptureCompare(ENCODER_B_CHANNEL);
  /* frequency computation */
  if (currentCaptureB > lastCaptureB) {
    frequencyMeasuredB = inputFreqB / (currentCaptureB - lastCaptureB);
  }
  else if (currentCaptureB <= lastCaptureB) {
    /* 0x1000 is max overflow value */
    frequencyMeasuredB = inputFreqB / (0x10000 + currentCaptureB - lastCaptureB);
  }
  lastCaptureB = currentCaptureB;
  rolloverComparecountB = 0;
}

/* In case of timer rollover, frequency is to low to be measured set value to 0
   To reduce minimum frequency, it is possible to increase prescaler. But this is at a cost of precision. */
void rolloverCaptureRisingB(void)
{
  rolloverComparecountB++;

  if (rolloverComparecountB > 1)
  {
    frequencyMeasuredB = 0;
  }
}