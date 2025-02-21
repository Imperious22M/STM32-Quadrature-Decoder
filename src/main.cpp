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

#define SPEED_UPDATE_RATE_HZ 1 // The rate at which speed is updated in Hz
#define CAN_UPDATE_RATE_HZ 10 // The rate at which CAN messages are transmited in Hz

// Revolutions per second of the encoder shaft
float revPerSec = 0;
// Previously stored Tick Count, initialized to zero upon reset
volatile uint16_t prevTickCnt = 0;

// Function signature of speed calculation interrupt
void calculateRevPerSec();

// Function signature to send speed and direction to CAN bus
void sendCANMessage();

// Create an QuadratureDecoder object, this wraps around the functionality of the decoder
// The Timer used MUST support Quadrature decoding, otherwise the functions will not work
QuadratureDecoder quadDecoder(TIM1, T1CH1_Pin, T1CH2_Pin);

// Hardware timer to send information to the CAN bus at a rate of 100 Hz
HardwareTimer *CANTimer = new HardwareTimer(TIM4);

// Determine the frequency of the T1CH1 pin
HardwareTimer *pulseTimer = new HardwareTimer(TIM3);
#define pin  PB_1

uint32_t channelRising, channelFalling;
volatile uint32_t FrequencyMeasured, DutycycleMeasured, LastPeriodCapture = 0, CurrentCapture, HighStateMeasured;
uint32_t input_freq = 0;
volatile uint32_t rolloverCompareCount = 0;

// Calculates Frequency from pulseTimer
void TIMINPUT_Capture_Rising_IT_callback(void)
{
  CurrentCapture = pulseTimer->getCaptureCompare(channelRising);
  /* frequency computation */
  if (CurrentCapture > LastPeriodCapture)
  {
    FrequencyMeasured = input_freq / (CurrentCapture - LastPeriodCapture);
  }
  else if (CurrentCapture <= LastPeriodCapture)
  {
    /*Compare to overflow value if we rolled over */
    FrequencyMeasured = input_freq / (pulseTimer->getOverflow() + CurrentCapture - LastPeriodCapture);
  }

  LastPeriodCapture = CurrentCapture;
  rolloverCompareCount = 0;
}

/* In case of timer rollover, frequency is to low to be measured set values to 0
   To reduce minimum frequency, it is possible to increase prescaler. But this is at a cost of precision. */
void Rollover_IT_callback(void)
{
  rolloverCompareCount++;

  if (rolloverCompareCount > 1)
  {
    FrequencyMeasured = 0;
  }
}

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

void setup()
{
  Serial.begin(9600);

  // Setup CAN update Timer
  CANTimer->setOverflow(CAN_UPDATE_RATE_HZ,HERTZ_FORMAT); // Send CAN messages at the set Hz rate
  CANTimer->attachInterrupt(sendCanMessage);

  // Automatically retrieve TIM instance and channelRising associated to pin
  // This is used to be compatible with all STM32 series automatically.
  TIM_TypeDef *Instance = (TIM_TypeDef *)pinmap_peripheral(pin, PinMap_PWM);
  channelRising = STM_PIN_CHANNEL(pinmap_function(pin, PinMap_PWM));
  channelRising = 3;

  // channelRisings come by pair for TIMER_INPUT_FREQ_DUTY_MEASUREMENT mode:
  // channelRising1 is associated to channelFalling and channelRising3 is associated with channelRising4
  switch (channelRising) {
    case 1:
      channelFalling = 2;
      break;
    case 2:
      channelFalling = 1;
      break;
    case 3:
      channelFalling = 4;
      break;
    case 4:
      channelFalling = 3;
      break;
  }

    // Configure rising edge detection to measure frequency
  pulseTimer->setMode(channelRising, TIMER_INPUT_CAPTURE_RISING, pin);

  // Experimental Delta T measure of 
  // With a PrescalerFactor = 1, the minimum frequency value to measure is : TIM counter clock / CCR MAX
  //  = (SystemCoreClock) / 65535
  // Example on Nucleo_L476RG with systemClock at 80MHz, the minimum frequency is around 1,2 khz
  // To reduce minimum frequency, it is possible to increase prescaler. But this is at a cost of precision.
  // The maximum frequency depends on processing of both interruptions and thus depend on board used
  // Example on Nucleo_L476RG with systemClock at 80MHz the interruptions processing is around 10 microseconds and thus Max frequency is around 100kHz
  uint32_t PrescalerFactor = 1;
  //pulseTimer->setPrescaleFactor(PrescalerFactor);
  //pulseTimer->setOverflow(0x10000); // Max Period value to have the largest possible time to detect rising edge and avoid timer rollover
  pulseTimer->setOverflow(20,HERTZ_FORMAT);
  pulseTimer->attachInterrupt(channelRising, TIMINPUT_Capture_Rising_IT_callback);
  pulseTimer->attachInterrupt(Rollover_IT_callback);


  pulseTimer->resume();

  // Compute this scale factor only once
  input_freq = pulseTimer->getTimerClkFreq() / pulseTimer->getPrescaleFactor();

  // Start speed timer
  //SpeedTimer->resume();
  //Start CAN timer
  //CANTimer->resume();

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
  Serial.println("~~~~");
  //Serial.println(pulseTimer->getTimerClkFreq());
  //Serial.println(pulseTimer->getPrescaleFactor());
  Serial.println((String)"Frequency = " + FrequencyMeasured);
  Serial.print("Revs/S: ");
  revPerSec = (float)FrequencyMeasured / teethNum;
  Serial.println(revPerSec);
  
  Serial.println("~~~~");
  delay(10);
}