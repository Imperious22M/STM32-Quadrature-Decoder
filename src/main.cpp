#include <Arduino.h>
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

// Quadrature decoders pinout and settings
#define T1CH1_Pin PA8 // Quadrature A1 encoder1Channel input pin
#define T1CH2_Pin PA9 // Quadrature B1 encoder1Channel input pin
//#define T2CH1_Pin PA15 // Quadrature A2 encoder1Channel input pin
//#define T2CH2_Pin PB3 // Quadrature B2 encoder1Channel input pin
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
#define SPEED_UPDATE_RATE_HZ 80
// Period of update rat2
double speedTimeRate =  1.0/SPEED_UPDATE_RATE_HZ; 
// Ticks per second of the encoder shaft #1
volatile double ticksPerSec1 = 0;
// Ticks per second of the encoder shaft #2
volatile double ticksPerSec2 = 0;
// previouslt measured ticks
volatile int prevTickCnt1 = 0;
volatile int prevTickCnt2 = 0;
// Tick Differentials
volatile int tickDiff1 = 0;
volatile int tickDiff2 = 0;

// Function signature of speed calculation interrupt
void calcTickPerSec();
// Function signature to send speed and direction to CAN bus
void sendCANMessage();

// Create a QuadratureDecoder object, this wraps around the functionality of the decoder
// The Timer used MUST support Quadrature decoding, otherwise the functions will not work
//QuadratureDecoder quadDecoder(TIM1, T1CH1_Pin, T1CH2_Pin);
QuadratureDecoder quadDecoder(TIM2, PA0, PA1);
// Second quadrature decoder
// TODO TAKE PB6 and PB7 out of this hardcoded call
QuadratureDecoder quadDecoder2(TIM4, PB6, PB7);
// Hardware timer to send information to the CAN bus at a rate of 100 Hz
//HardwareTimer *CANTimer = new HardwareTimer(TIMER_TONE);
// Timer Timer
HardwareTimer *SpeedTimer = new HardwareTimer(TIM5);
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

// Quadrature decoding functions

// Calculates the differenc in Ticks and the corresponding TPS
// using a rolling average
// Meant to be used as an ISR
void calcTickDiff(){
  // Current count of each shaft
  uint32_t curTickCnt1 = quadDecoder.getCountQuad(TICK_FORMAT);
  uint32_t curTickCnt2 = quadDecoder2.getCountQuad(TICK_FORMAT);
  tickDiff1 = curTickCnt1 - prevTickCnt1;
  tickDiff2 = curTickCnt2 - prevTickCnt2;

  // Experimental rolling average
  // Takes the number of ticks that have been received and averages them over
  // the number of readings set. TPS is set to 0 when all the numbers inside the
  // rolling average are equal to 0, which means the time to reset is equal to:
  // speedTimeRate*number of readings
  static int avgTick1[SPEED_UPDATE_RATE_HZ] = {0};
  static int avgTick2[SPEED_UPDATE_RATE_HZ] = {0};
  static int indexTick1 = 0;
  static int indexTick2 = 0;
  const int avrgArraySize = SPEED_UPDATE_RATE_HZ;

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
  int periodsTicked1=0;
  int periodsTicked2=0;
  for(int x=0;x<avrgArraySize;x++){
    if(avgTick1[x]!=0){
      totalTicks1+=avgTick1[x];
      periodsTicked1++;
    }
    if(avgTick2[x]!=0){
      totalTicks2+=avgTick2[x];
      periodsTicked2++;
    }
  }
  // Calculate TPS for both tick 1 and tick 2
  if(periodsTicked1>0){
    //ticksPerSec1 = (float)totalTicks1/(periodsTicked1*speedTimeRate);
    ticksPerSec1 = ((double)totalTicks1*speedTimeRate)/(periodsTicked1);
  }else{
    ticksPerSec1 = 0;
  }
  if(periodsTicked2>0){
    ticksPerSec2 = (double)totalTicks2/(periodsTicked2*speedTimeRate);
  }else{
    ticksPerSec2 = 0;
  }

  // Assign previous ticks to current ticks
  prevTickCnt1 = curTickCnt1;
  prevTickCnt2 = curTickCnt2;
}

// Test TPS function at meant to be used at a 1 Hz Rate
void calc1Sec(){
uint32_t curTickCnt1 = quadDecoder.getCountQuad(TICK_FORMAT);
static int prevTickCnt_local = 0;
int tickDiff = curTickCnt1 - prevTickCnt_local;

Serial.println("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
Serial.print("1 Sec TPS:");
Serial.println(tickDiff);
Serial.println(millis());
Serial.println("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
prevTickCnt_local = curTickCnt1;
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

  pinMode(PB6, INPUT);
  pinMode(PB7, INPUT);
// configure GPIO PA0 & PA1 as inputs for Encoder
    RCC->AHB1ENR |= 0x00000001;  // Enable clock for GPIOA
 
    GPIOA->MODER   |= GPIO_MODER_MODER0_1 | GPIO_MODER_MODER1_1 ;           //PA0 & PA1 as Alternate Function   /*!< GPIO port mode register,               Address offset: 0x00      */
    GPIOA->OTYPER  |= GPIO_OTYPER_OT_0 | GPIO_OTYPER_OT_1 ;                 //PA0 & PA1 as Inputs               /*!< GPIO port output type register,        Address offset: 0x04      */
    GPIOA->OSPEEDR |= 0x00000011;//|= GPIO_OSPEEDER_OSPEEDR0 | GPIO_OSPEEDER_OSPEEDR1 ;     // Low speed                        /*!< GPIO port output speed register,       Address offset: 0x08      */
    GPIOA->PUPDR   |= GPIO_PUPDR_PUPDR0_1 | GPIO_PUPDR_PUPDR1_1 ;           // Pull Down                        /*!< GPIO port pull-up/pull-down register,  Address offset: 0x0C      */
    GPIOA->AFR[0]  |= 0x00000011 ;                                          //  AF01 for PA0 & PA1              /*!< GPIO alternate function registers,     Address offset: 0x20-0x24 */
    GPIOA->AFR[1]  |= 0x00000000 ;                                          //                                  /*!< GPIO alternate function registers,     Address offset: 0x20-0x24 */
   
    // configure TIM2 as Encoder input
    RCC->APB1ENR |= 0x00000001;  // Enable clock for TIM2
 
    TIM2->CR1   = 0x0001;     // CEN(Counter ENable)='1'     < TIM control register 1
    TIM2->SMCR  = 0x0003;     // SMS='011' (Encoder mode 3)  < TIM slave mode control register
    TIM2->CCMR1 = 0x0101;     // CC1S='01' CC2S='01'         < TIM capture/compare mode register 1
    TIM2->CCMR2 = 0x0000;     //                             < TIM capture/compare mode register 2
    TIM2->CCER  = 0x0011;     // CC1P CC2P                   < TIM capture/compare enable register
    TIM2->PSC   = 0x0000;     // Prescaler = (0+1)           < TIM prescaler
    TIM2->ARR   = 0xffffffff; // reload at 0xfffffff         < TIM auto-reload register
    //Tim2->BDTR
    TIM2->CNT = 0x0000;  //reset the counter before we use it 
  // Setup CAN update Timer
  //CANTimer->setOverflow(1,HERTZ_FORMAT); // Send CAN messages at the set Hz rate
  //CANTimer->setOverflow(62499);
  //CANTimer->setPrescaleFactor(1151);
  //CANTimer->attachInterrupt(calc1Sec);
  //Start CAN timer
  //CANTimer->resume();

  // Start speed timer
  SpeedTimer->setOverflow(SPEED_UPDATE_RATE_HZ,HERTZ_FORMAT); // Update speed at the set Hz rate
  speedTimeRate = ((double)SpeedTimer->getTimerClkFreq() / (double)SpeedTimer->getPrescaleFactor()/(double)SpeedTimer->getOverflow());

 // while (true)
 // {
 //   Serial.println(speedTimeRate, 10);
 //   delay(1000);
 // }
  

  //SpeedTimer->setOverflow(59999);
  //SpeedTimer->setPrescaleFactor(11);
  //SpeedTimer->attachInterrupt(calcTickPerSec_1); // Attach the speed calculation function to the timer
  SpeedTimer->attachInterrupt(calcTickDiff); // Attach the speed calculation function to the timer
  //SpeedTimer->attachInterrupt(calcTickPerSec_1); // Attach the speed calculation function to the timer
  SpeedTimer->resume();

  // CAN Bus module configuration
  pinMode(CS_PIN,OUTPUT);
  // The amazon CAN modules have 8 Mhz crystals by default
  //mcp.setClockFrequency(8e6);
  // If CAN controller is not found, fail to read encoders
 // if (!mcp.begin(CAN_BAUDRATE)) {
 //   while(1) {
 //   delay(1000);
 //   Serial.println("Error initializing MCP2515.");
 //   }
 // }

  //pinMode(T3CH1_Pin,INPUT);
  //pinMode(T2CH3_Pin);

  // Frequency measure of the single encoder1Channel decoders 
  // in the front wheels
  singleDecTimer1 = new HardwareTimer(TIM3);
  //pinMode(PB1,INPUT);
  // Configure rising edge detection to measure frequency
  singleDecTimer1->setMode(encoder1Channel, TIMER_INPUT_CAPTURE_RISING, T3CH1_Pin);
  //singleDecTimer1->setMode(4, TIMER_INPUT_CAPTURE_RISING, PB1);
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
  Serial.println(quadDecoder.getCountQuad(TICK_FORMAT));
  //Serial.print("Dir1:");
  //Serial.println(quadDecoder.getDirBit());
  Serial.print("TPS1:");
  Serial.println(ticksPerSec1);
  //Serial.print("Count_2:");
  //Serial.println(quadDecoder2.getCountQuad(TICK_FORMAT));
  Serial.print("TPS2:");
  Serial.println(ticksPerSec2);
    /* Print frequency measured on Serial monitor every seconds */
  Serial.println((String)"Freq1:"+frequencyMeasured1);
  Serial.println((String)"RPS1:"+(float)frequencyMeasured1/teethNum);
  Serial.println((String)"Freq2:"+frequencyMeasured2);
  Serial.println((String)"RPS2:"+(float)frequencyMeasured2/teethNum);
  delay(SPEED_UPDATE_RATE_HZ);
}