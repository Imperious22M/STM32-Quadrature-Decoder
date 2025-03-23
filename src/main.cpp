#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <quadRegisterMethod.h>

/*
  Quadrature Decoder example using hardware timer 1 (Ch1 and Ch2)
  4x mode, with hardware filtering and edge detection
*/

#if !defined(STM32_CORE_VERSION) || (STM32_CORE_VERSION  < 0x01090000)
#error "Due to API change, this sketch is compatible with STM32_CORE_VERSION  >= 0x01090000"
#endif

// Quadrature decoders pinout and settings
#define ticksPerRevolution 50*4 // Number of "ticks" per revolution
#define teethNum 25 // number of teeth the gear has

// Single decoder pinout and settings
#define T3CH1_Pin PB4 // Single encoder A
const uint32_t encoder1Channel = 1;
volatile uint32_t frequencyMeasured1, lastCapture1 = -1, currentCapture1;
uint32_t inputFreq1 = 0;
volatile uint32_t rolloverComparecount1 = 0;
HardwareTimer *singleDecTimer1;
#define T2CH3_Pin PA2 // Single encoder B
const uint32_t encoder2Channel = 3;
volatile uint32_t frequencyMeasured2, lastCapture2 = -1, currentCapture2;
uint32_t inputFreq2 = 0;
volatile uint32_t rolloverComparecount2 = 0;
HardwareTimer *singleDecTimer2;


// Speed update rate in HZ
#define SPEED_UPDATE_RATE_HZ 20
// Period of update rat2
double tpsTimerate =  1.0/SPEED_UPDATE_RATE_HZ; 
// Ticks per second of the encoder shaft #1
volatile double ticksPerSec1 = 0;
// Ticks per second of the encoder shaft #2
volatile double ticksPerSec2 = 0;
// previouslt measured ticks
volatile uint32_t prevTickCnt1 = 0;
volatile uint32_t prevTickCnt2 = 0;
// Tick Differentials
volatile long int tickDiff1 = 0;
volatile long int tickDiff2 = 0;
// Quadrature direction
volatile bool dir1 = 0;
volatile bool dir2 = 0;


// Function signature of speed calculation interrupt
void calcTickPerSec();
// Function signature to send speed and direction to CAN bus
void sendCANMessage();

// Timer to obtain TPS at a set interval
HardwareTimer *tpsTimer = new HardwareTimer(TIM5);

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

  // Experimental rolling average
  // Takes the number of ticks that have been received and averages them over
  // the number of readings set. TPS is set to 0 when all the numbers inside the
  // rolling average are equal to 0, which means the time to reset is equal to:
  // tpsTimerate*number of readings
  static int avgTick1[SPEED_UPDATE_RATE_HZ] = {0};
  static int avgTick2[SPEED_UPDATE_RATE_HZ] = {0};
  static int indexTick1 = 0;
  static int indexTick2 = 0;
  const int avrgArraySize = SPEED_UPDATE_RATE_HZ;
  int dir1Loc = 0;
  int dir2Loc = 0;

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
    if(ticksPerSec1>0){
      dir1Loc = 0;
    }else{
      dir1Loc = 1;
    }
  }else{
    ticksPerSec1 = 0;
  }
  if(periodsTicked2>0){
    //ticksPerSec2 = (double)totalTicks2/(periodsTicked2*tpsTimerate);
    ticksPerSec2 = ((double)totalTicks2*tpsTimerate)/(periodsTicked2);
    if(ticksPerSec2>0){
      dir2Loc = 0;
    }else{
      dir2Loc = 1;
    }
  }else{
    ticksPerSec2 = 0;
  }

  // Account for overflow/underflow of counters
  if(dir1==0 && dir1Loc==1 && (-(tickDiff1)+curTickCnt1)>=UINT16_MAX){
    // If direction changes and ticks read in total are greater are equal to 
    //Serial.println("OVERFLOW_1");
    Serial.println(tickDiff1);
    Serial.println(curTickCnt1);
    Serial.println(-(tickDiff1)+curTickCnt1);
    delay(3000);
  }
  if(dir1==1 && dir1Loc==0 && (tickDiff1+curTickCnt1)>=UINT16_MAX){
    //Serial.println("UNDERFLOW_1");
    Serial.println(tickDiff1);
    Serial.println(curTickCnt1);
    Serial.println(tickDiff1+curTickCnt1);
    delay(3000);
  }

  // Update diretion
  dir1 = dir1Loc;
  dir2 = dir2Loc;
  // Assign previous ticks to current ticks
  prevTickCnt1 = curTickCnt1;
  prevTickCnt2 = curTickCnt2;
}

// Single encoder decoding functions
void inputCaptureRising1(void)
{
  currentCapture1 = singleDecTimer1->getCaptureCompare(encoder1Channel);
  /* frequency computation */
  if (currentCapture1 > lastCapture1) {
    frequencyMeasured1 = inputFreq1 / (currentCapture1 - lastCapture1);
  }
  else if (currentCapture1 <= lastCapture1) {
    /* 0x1000 is max overflow value */
    frequencyMeasured1 = inputFreq1 / (0x10000 + currentCapture1 - lastCapture1);
  }
  lastCapture1 = currentCapture1;
  rolloverComparecount1 = 0;
}

/* In case of timer rollover, frequency is to low to be measured set value to 0
   To reduce minimum frequency, it is possible to increase prescaler. But this is at a cost of precision. */
void rolloverCaptureRising1(void)
{
  rolloverComparecount1++;

  if (rolloverComparecount1 > 1)
  {
    frequencyMeasured1 = 0;
  }

  rolloverComparecount2++;

  if (rolloverComparecount2 > 1)
  {
    frequencyMeasured2 = 0;
  }

}

void inputCaptureRising2_1(void)
{
  currentCapture1 = singleDecTimer1->getCaptureCompare(4);
  /* frequency computation */
  if (currentCapture2 > lastCapture2) {
    frequencyMeasured2 = inputFreq2 / (currentCapture2 - lastCapture2);
  }
  else if (currentCapture2 <= lastCapture2) {
    /* 0x1000 is max overflow value */
    frequencyMeasured2 = inputFreq2 / (0x10000 + currentCapture2 - lastCapture2);
  }
  lastCapture2 = currentCapture2;
  rolloverComparecount2 = 0;
}
void inputCaptureRising2(void)
{
  Serial.println("WEEEEE");
  currentCapture2 = singleDecTimer2->getCaptureCompare(encoder2Channel);
  /* frequency computation */
  if (currentCapture2 > lastCapture2) {
    frequencyMeasured2 = inputFreq2 / (currentCapture2 - lastCapture2);
  }
  else if (currentCapture2 <= lastCapture2) {
    /* 0x1000 is max overflow value */
    frequencyMeasured2 = inputFreq2 / (0x10000 + currentCapture2 - lastCapture2);
  }
  lastCapture2 = currentCapture2;
  rolloverComparecount2 = 0;
}

/* In case of timer rollover, frequency is to low to be measured set value to 0
   To reduce minimum frequency, it is possible to increase prescaler. But this is at a cost of precision. */
void rolloverCaptureRising2(void)
{
  rolloverComparecount2++;

  if (rolloverComparecount2 > 1)
  {
    frequencyMeasured2 = 0;
  }

}

void setup()
{
  Serial.begin(9600);

  // Configure quadrature decoders
  setupQuadTimers();

  // Start speed timer
  tpsTimer->setOverflow(SPEED_UPDATE_RATE_HZ,HERTZ_FORMAT); // Update speed at the set Hz rate
  tpsTimer->attachInterrupt(calcTickDiff); // Attach the speed calculation function to the timer
  tpsTimer->resume();
  // Calculate the actual HZ rate of the timer (the requested is accurate up to 10^-3)
  tpsTimerate = ((double)tpsTimer->getTimerClkFreq() / (double)tpsTimer->getPrescaleFactor()/(double)tpsTimer->getOverflow());

  // Frequency measure of the single encoder1Channel decoders 
  // in the front wheels
  singleDecTimer1 = new HardwareTimer(TIM9);
  // Configure rising edge detection to measure frequency
  singleDecTimer1->setMode(encoder1Channel, TIMER_INPUT_CAPTURE_RISING, T3CH1_Pin);
  // Set prescaler and overflow
  // This determines the accuracy and range of the frequency measure
  uint32_t PrescalerFactor = 1151;
  singleDecTimer1->setPrescaleFactor(PrescalerFactor);
  singleDecTimer1->setOverflow(62499); // Max Period value to have the largest possible time to detect rising edge and avoid timer rollover
  singleDecTimer1->attachInterrupt(encoder1Channel, inputCaptureRising1);
  //singleDecTimer1->attachInterrupt(4, inputCaptureRising2_1);
  singleDecTimer1->attachInterrupt(rolloverCaptureRising1);
  singleDecTimer1->resume();
    // Compute this scale factor only once
  inputFreq1 = singleDecTimer1->getTimerClkFreq() / singleDecTimer1->getPrescaleFactor();

  //inputFreq2 = inputFreq1;
  //singleDecTimer2 = new HardwareTimer(TIM2);
  //// Configure rising edge detection to measure frequency
  //singleDecTimer2->setMode(encoder2Channel, TIMER_INPUT_CAPTURE_RISING, T2CH3_Pin);
  //// Set prescaler and overflow
  //// This determines the accuracy and range of the frequency measure
  //singleDecTimer2->setPrescaleFactor(PrescalerFactor);
  //singleDecTimer2->setOverflow(62499); // Max Period value to have the largest possible time to detect rising edge and avoid timer rollover
  //singleDecTimer2->attachInterrupt(encoder2Channel, inputCaptureRising2);
  //singleDecTimer2->attachInterrupt(rolloverCaptureRising2);
  ////singleDecTimer2->resume();
  //  // Compute this scale factor only once
  //inputFreq2 = singleDecTimer2->getTimerClkFreq() / singleDecTimer2->getPrescaleFactor();
}

void loop()
{

  //calcTickPerSec_1();
  Serial.print("Count_1:");
  Serial.println(readQuadA());
  //Serial.print("Dir1:");
  //Serial.println(quadDecoder.getDirBit());
  Serial.print("TPS1:");
  Serial.println(ticksPerSec1);
  //Serial.print("Count_2:");
  //Serial.println(readQuadB());
  //Serial.print("TPS2:");
  //Serial.println(ticksPerSec2);
    /* Print frequency measured on Serial monitor every seconds */
  //Serial.println((String)"Freq1:"+frequencyMeasured1);
  //Serial.println((String)"RPS1:"+(float)frequencyMeasured1/teethNum);
  //Serial.println((String)"Freq2:"+frequencyMeasured2);
  //Serial.println((String)"RPS2:"+(float)frequencyMeasured2/teethNum);
  delay(100);
}