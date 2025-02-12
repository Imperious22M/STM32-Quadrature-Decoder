#include <Arduino.h>
#include <stm32yyxx_ll_tim.h>
/*
  Quadrature Decoder example using hardware timer 1 (Ch1 and Ch2)
  4x mode, with hardware filtering and edge detection (TODO)
*/

#if !defined(STM32_CORE_VERSION) || (STM32_CORE_VERSION  < 0x01090000)
#error "Due to API change, this sketch is compatible with STM32_CORE_VERSION  >= 0x01090000"
#endif

#define T1CH1_Pin PA8 // Quadrature A channel input pin
#define T1CH2_Pin PA9 // Quadrature B channel input pin

timerObj_t _timerObj;
#define  TIMER_CHANNELS 4    // channel 5 and channel 6 are not considered here has they don't have gpio output and they don't have interrupt
#define MAX_RELOAD ((1 << 16) - 1) // Currently even 32b timers are used as 16b to have generic behavior

void setupTimer(TIM_TypeDef *instance)
{

  _timerObj.handle.Instance = instance;
  _timerObj.handle.Channel = HAL_TIM_ACTIVE_CHANNEL_CLEARED;
  _timerObj.handle.hdma[0] = NULL;
  _timerObj.handle.hdma[1] = NULL;
  _timerObj.handle.hdma[2] = NULL;
  _timerObj.handle.hdma[3] = NULL;
  _timerObj.handle.hdma[4] = NULL;
  _timerObj.handle.hdma[5] = NULL;
  _timerObj.handle.hdma[6] = NULL;
  _timerObj.handle.Lock = HAL_UNLOCKED;
  _timerObj.handle.State = HAL_TIM_STATE_RESET;

  // Not used for this project
  //_timerObj.preemptPriority = TIM_IRQ_PRIO;
  //_timerObj.subPriority = TIM_IRQ_SUBPRIO;

  /* Enable timer clock. Even if it is also done in HAL_TIM_Base_MspInit(),
     it is done there so that it is possible to write registers right now */
  enableTimerClock(&(_timerObj.handle));

  // Initialize channel mode and complementary
  //for (int i = 0; i < TIMER_CHANNELS; i++) {
  //  __ChannelsUsed[i] = 0x00;
  //  _ChannelMode[i] = TIMER_OUTPUT_DISABLED;
  //}

  /* Configure timer with some default values */
  _timerObj.handle.Init.Prescaler = 0;
  _timerObj.handle.Init.Period = MAX_RELOAD;
  _timerObj.handle.Init.CounterMode = TIM_COUNTERMODE_UP;
  _timerObj.handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  #if defined(TIM_RCR_REP)
    _timerObj.handle.Init.RepetitionCounter = 0;
  #endif
  _timerObj.handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  
  // Initialize the TIM low level resources
  HAL_TIM_Base_Init(&(_timerObj.handle));
}

uint32_t getCountQuad(TimerFormat_t format)
{
  uint32_t CNT_RegisterValue = LL_TIM_GetCounter(_timerObj.handle.Instance);
  uint32_t Prescalerfactor = LL_TIM_GetPrescaler(_timerObj.handle.Instance) + 1;
  uint32_t return_value;
  switch (format) {
    case TICK_FORMAT:
    default :
      return_value = CNT_RegisterValue;
      break;
  }
  return return_value;
}

void setup()
{
  Serial.begin(9600);

  setupTimer(TIM1);
  
  // We will be using the HAL of the STM32 instead of the Arduino Framework
  // to set up a Hardware decoder. Please refer to the documentation for more information
  TIM_Encoder_InitTypeDef TIM_EncoderInitStruct;
  TIM_EncoderInitStruct.EncoderMode = TIM_ENCODERMODE_TI12;
  TIM_EncoderInitStruct.IC1Polarity = TIM_ICPOLARITY_RISING;
  TIM_EncoderInitStruct.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  TIM_EncoderInitStruct.IC1Prescaler = TIM_ICPSC_DIV1;
  TIM_EncoderInitStruct.IC1Filter = 0;
  TIM_EncoderInitStruct.IC2Polarity = TIM_ICPOLARITY_RISING;
  TIM_EncoderInitStruct.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  TIM_EncoderInitStruct.IC2Prescaler = TIM_ICPSC_DIV1;
  TIM_EncoderInitStruct.IC2Filter = 0;

  // In the STM32 Arduino Core, configuring them as Inputs is sufficient for them 
  // to be used as with the Timers
  pinMode(T1CH1_Pin, INPUT);
  pinMode(T1CH2_Pin, INPUT);

  // Configure the TIM in the desired functionality
  HAL_TIM_Encoder_Init(&(_timerObj.handle), &TIM_EncoderInitStruct); 
  // Start the Hardware Timers
  HAL_TIM_Encoder_Start(&(_timerObj.handle), TIM_CHANNEL_ALL);
}


void loop()
{
  Serial.println(getCountQuad(TICK_FORMAT));
  Serial.println("~~~~");
  delay(1000);
}