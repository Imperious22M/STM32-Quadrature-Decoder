#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <quadRegisterMethod.h>
#include <singleQuadISR.h>
#include <canHelpers.h>

/*
  Quadrature Decoder example using hardware timer 1 (Ch1 and Ch2)
  4x mode, with hardware filtering and edge detection
*/

#if !defined(STM32_CORE_VERSION) || (STM32_CORE_VERSION  < 0x01090000)
#error "Due to API change, this sketch is compatible with STM32_CORE_VERSION  >= 0x01090000"
#endif

// Quadrature decoders settings
#define ticksPerRevolution 50*4 // Number of "ticks" per revolution
#define teethNum 25 // number of teeth the gear has
// Speed update rate in HZ
#define TPS_REFRESH_RATE 100
// Period of TPS update rate
double tpsTimerate =  1.0/TPS_REFRESH_RATE; 
// Ticks per second of the encoder shaft #1
volatile double ticksPerSec1 = 0;
// Ticks per second of the encoder shaft #2
volatile double ticksPerSec2 = 0;
// previouslt measured ticks
volatile uint32_t prevTickCnt1 = 0x8000;
volatile uint32_t prevTickCnt2 = 0x8000;
// Tick Differentials
volatile long int tickDiff1 = 0;
volatile long int tickDiff2 = 0;
// Quadrature direction
volatile bool dir1 = 0;
volatile bool dir2 = 0;
// Quadrature count
long int quadCnt1 = 0;
long int quadCnt2 = 0;

// Function signature of speed calculation interrupt
void calcTickPerSec();
// Function signature to send speed and direction to CAN bus
void sendCANMessage();

// Timer to obtain TPS at a set interval
HardwareTimer *tpsTimer = new HardwareTimer(TIM2);

// Calculates the differenc in Ticks and the corresponding TPS
// using a rolling average
// Meant to be used as an ISR
void calcTickDiff(){
  // Current count of each shaft
  // Treat the encoder as 16-bit values
  uint32_t curTickCnt1 = readQuadA();
  uint32_t curTickCnt2 = readQuadB();
  tickDiff1 = curTickCnt1 - prevTickCnt1;
  tickDiff2 = curTickCnt2 - prevTickCnt2;
  // update quadrature decoder counts
  quadCnt1+=tickDiff1;
  quadCnt2+=tickDiff2;
  
  // Experimental rolling average
  // Takes the number of ticks that have been received and averages them over
  // the number of readings set. TPS is set to 0 when all the numbers inside the
  // rolling average are equal to 0, which means the time to reset is equal to:
  // tpsTimerate*number of readings
  static int avgTick1[TPS_REFRESH_RATE] = {0};
  static int avgTick2[TPS_REFRESH_RATE] = {0};
  static int indexTick1 = 0;
  static int indexTick2 = 0;
  const int avrgArraySize = TPS_REFRESH_RATE;

  if(indexTick1==avrgArraySize){
    indexTick1=0;
  }
  if(indexTick2==avrgArraySize){
    indexTick2=0;
  }
  avgTick1[indexTick1] = tickDiff1;
  avgTick2[indexTick2] = tickDiff2;
  indexTick1++;
  indexTick2++;
  // Calculate the average for both tick 1 and tick 2
  int totalTicks1=0;
  int totalTicks2=0;
  uint32_t periodsTicked1=0;
  uint32_t periodsTicked2=0;
  for(int x=0;x<avrgArraySize;x++){
    if(avgTick1[x]!=0){
      totalTicks1+=avgTick1[x];
    }
      periodsTicked1++;

    if(avgTick2[x]!=0){
      totalTicks2+=avgTick2[x];
    }
      periodsTicked2++;
  }
  // Calculate TPS for both tick 1 and tick 2
  if(periodsTicked1>0){
    //ticksPerSec1 = (float)totalTicks1/(periodsTicked1*tpsTimerate); 
    ticksPerSec1 = ((double)totalTicks1*tpsTimerate)/(periodsTicked1);
  }else{
    ticksPerSec1 = 0;
  }
  if(periodsTicked2>0){
    //ticksPerSec2 = (double)totalTicks2/(periodsTicked2*tpsTimerate);
    ticksPerSec2 = ((double)totalTicks2*tpsTimerate)/(periodsTicked2);
  }else{
    ticksPerSec2 = 0;
  }

  // Update diretion
  dir1 = (ticksPerSec1>0&&ticksPerSec1!=0)? 0 : 1;
  dir2 = (ticksPerSec1>0&&ticksPerSec2!=0)? 0 : 1;
  // Assign previous ticks to current ticks
  //prevTickCnt1 = curTickCnt1;
  //prevTickCnt2 = curTickCnt2;
}


void setup()
{
  Serial.begin(115200);

  // Configure quadrature decoders
  setupQuadTimers();

  // Setup CAN Module
  setupCAN();

  // Start TPS timer for the quadrature decoders
  tpsTimer->setOverflow(TPS_REFRESH_RATE,HERTZ_FORMAT); // Update speed at the set Hz rate
  tpsTimer->attachInterrupt(calcTickDiff); // Attach the speed calculation function to the timer
  tpsTimer->resume();
  // Calculate the actual HZ rate of the timer (the requested is accurate up to 10^-3)
  tpsTimerate = ((double)tpsTimer->getTimerClkFreq() / (double)tpsTimer->getPrescaleFactor()/(double)tpsTimer->getOverflow());

  // Frequency measure of the single channel encoders
  singleDecTimerA = new HardwareTimer(TIM1);
  // Configure rising edge detection to measure frequency
  singleDecTimerA->setMode(ENCODER_A_CHANNEL, TIMER_INPUT_CAPTURE_RISING, SINGLE_ENCODER_A_PIN);
  singleDecTimerA->setOverflow(SINGLE_ENCODER_REFRESH_RATE,HERTZ_FORMAT); // 1 HZ Refresh Rate 
  singleDecTimerA->attachInterrupt(ENCODER_A_CHANNEL, inputCaptureRisingA);
  singleDecTimerA->attachInterrupt(rolloverCaptureRisingA);
  singleDecTimerA->resume();
    // Compute this scale factor only once
  inputFreqA = singleDecTimerA->getTimerClkFreq() / singleDecTimerA->getPrescaleFactor();

  singleDecTimerB = new HardwareTimer(TIM4);
  // Configure rising edge detection to measure frequency
  singleDecTimerB->setMode(ENCODER_B_CHANNEL, TIMER_INPUT_CAPTURE_RISING, SINGLE_ENCODER_B_PIN);
  singleDecTimerB->setOverflow(SINGLE_ENCODER_REFRESH_RATE,HERTZ_FORMAT); // 1 HZ Refresh Rate 
  singleDecTimerB->attachInterrupt(ENCODER_B_CHANNEL, inputCaptureRisingB);
  singleDecTimerB->attachInterrupt(rolloverCaptureRisingB);
  singleDecTimerB->resume();
    // Compute this scale factor only once
  inputFreqB = singleDecTimerB->getTimerClkFreq() / singleDecTimerB->getPrescaleFactor();

}

void loop()
{
  Serial.print("Count_1:");
  Serial.println(quadCnt1);
  //Serial.println(TIM5->CNT);
  Serial.print("Count_2:");
  Serial.println(quadCnt2);
  //Serial.println(TIM3->CNT);
  Serial.print("Dir2:");
  Serial.println(dir2);
  Serial.print("TPS1:");
  Serial.println(ticksPerSec1);
  Serial.print("TPS2:");
  Serial.println(ticksPerSec2);
  Serial.println((String)"Freq1:"+frequencyMeasuredA);
  Serial.println((String)"RPS1:"+(float)frequencyMeasuredA/teethNum);
  Serial.println((String)"Freq2:"+frequencyMeasuredB);
  Serial.println((String)"RPS2:"+(float)frequencyMeasuredB/teethNum);
  delay(100);
}