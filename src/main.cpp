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
#define teethNum 25 // number of teeth the gear has
#define ticksPerRevolution teethNum*4 // Number of "ticks" per revolution
// Speed update rate in HZ
#define TPS_REFRESH_RATE 1000
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

// Timers used
// TIM1 -> Single Encoder A timer (HZA)
// TIM2 -> TPS interval timer
// TIM3 -> QuadB (TPS2)
// TIM4 -> Single Encoder B timer (HZB)
// TIM5 -> QuadA (TPS2)
// TIM9 -> CAN Message Interrupt (TEST!)

void sendCanMessage();

// Function signature of speed calculation interrupt
void calcTickPerSec();

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
  
  // Update global CAN variables
  //TPS1 = ticksPerSec1;
  //TPS2 = ticksPerSec2;
  //HZA = frequencyMeasuredA;
  //HZB = frequencyMeasuredB;
  //sendCanMessage();
}

// Send a CAN message with speed and direction
// TPS1 = back left wheel, TPS2 = back right wheel
// HZ1 = front left wheel, HZ2 = front right wheel
void sendCanMessage(){
  //canTransceiver.beginPacket(0x12);
  //canTransceiver.write('a');
  //canTransceiver.write('b');
  //canTransceiver.write('c');
  //canTransceiver.write('d');
  //canTransceiver.write('e');
  //canTransceiver.endPacket();
  ////canTransceiver.beginPacket(backLeftCANAddress);
  ////canTransceiver.printf("%i",ticksPerSec1);
  ////canTransceiver.endPacket();
  ////canTransceiver.beginPacket(backRightCANAddress);
  ////canTransceiver.printf("%i",ticksPerSec2);
  ////canTransceiver.endPacket();
  ////canTransceiver.beginPacket(frontLeftCANAddress);
  ////canTransceiver.printf("%i",frequencyMeasuredA);
  ////canTransceiver.endPacket();
  ////canTransceiver.beginPacket(frontRightCANAddress);
  ////canTransceiver.printf("%i",frequencyMeasuredB);
  ////canTransceiver.endPacket();
  //Serial.println("CAN SENT!");
  //Serial.println(millis())  
  byte data[8] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07}; 
  // send data:  ID = 0x100, Standard CAN Frame, Data length = 8 bytes, 'data' = array of data bytes to send
  CAN0.enOneShotTX();
  byte sndStat = CAN0.sendMsgBuf(0x100, 0, 8, data);
  if(sndStat == CAN_OK){
    Serial.println("Message Sent Successfully!");
  } else {
    Serial.println("Error Sending Message...");
  };
}

void setup()
{
  Serial.begin(115200);

  // Configure quadrature decoders
  setupQuadTimers();

  // Setup CAN Module

  setupCAN();
  //CHECK!!!!
  //canTransceiver.setClockFrequency(8e6);
  //pinMode(CS_CAN,OUTPUT);
  // while(!Serial) delay(10);
  // Serial.println("MCP2515 Sender test!");
  // if (!canTransceiver.begin(CAN_BAUDRATE)) {
  //   Serial.println("Error initializing MCP2515.");
  //   pinMode(LED_BUILTIN,OUTPUT);
  //   while(1) {
  //      digitalWrite(LED_BUILTIN,HIGH);
  //      delay(100);
  //      digitalWrite(LED_BUILTIN,LOW);
  //      delay(100);
  //   }
  // }
  // Start TPS timer for the quadrature decoders
  tpsTimer->setOverflow(TPS_REFRESH_RATE,HERTZ_FORMAT); // Update speed at the set Hz rate
  tpsTimer->attachInterrupt(calcTickDiff); // Attach the speed calculation function to the timer
  tpsTimer->resume();
  // Calculate the actual HZ rate of the timer (the requested is accurate up to 10^-3)
  tpsTimerate = ((double)tpsTimer->getTimerClkFreq() / (double)tpsTimer->getPrescaleFactor()/(double)tpsTimer->getOverflow());

  // Setup CAN Timer
  //canTimer->setOverflow(CAN_UPDATE_RATE_HZ,HERTZ_FORMAT); // Update speed at the set Hz rate
  //canTimer->attachInterrupt(sendCanMessage); // Attach the speed calculation function to the timer
  //canTimer->resume();

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
  Serial.print("Count_2:");
  Serial.println(quadCnt2);
  Serial.print("Dir2:");
  Serial.println(dir2);
  Serial.print("TPS1 (BL):");
  Serial.println(ticksPerSec1);
  Serial.print("TPS2 (BR):");
  Serial.println(ticksPerSec2);
  Serial.print("M/S1:");
  Serial.println((ticksPerSec1/(ticksPerRevolution))*1.725);
  Serial.print("M/S2:");
  Serial.println((ticksPerSec2/(ticksPerRevolution))*1.725);
  Serial.println((String)"Freq1 (FL):"+frequencyMeasuredA);
  Serial.println((String)"RPS1:"+(float)frequencyMeasuredA/teethNum);
  Serial.println((String)"Freq2 (FR):"+frequencyMeasuredB);
  Serial.println((String)"RPS2:"+(float)frequencyMeasuredB/teethNum);
  sendCanMessage();
  delay(1000);

}