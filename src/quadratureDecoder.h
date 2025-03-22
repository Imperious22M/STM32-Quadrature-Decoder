
#include <Arduino.h>
#include <stm32yyxx_ll_tim.h>

#define  TIMER_CHANNELS 4    // channel 5 and channel 6 are not considered here has they don't have gpio output and they don't have interrupt
#define MAX_RELOAD ((1 << 16) - 1) // Currently even 32b timers are used as 16b to have generic behavior

// Example on configuring TIM2 as a qudrature encoder using registers
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

class QuadratureDecoder {
  public:
    
    QuadratureDecoder(TIM_TypeDef *TIMInstance, uint32_t CH1PinArg, uint32_t CH2Pin) : CH1Pin(CH1Pin), CH2Pin(CH2Pin)
    {
      // Set the instance of the timer
      //_instance = TIMInstance;
      // Set up the basic settings of the timer hardware
      MyTim = new HardwareTimer(TIMInstance);
      
      basicTimerSetup(TIMInstance);
      // Set up the quadrature decoder settings
      quadratureSetup();

      MyTim->resume();
      // Set up the pins as inputs
      pinInputSetup(CH1Pin, CH2Pin);
    }

    // Returns the current count of the "CNT" register of the timer
    // in units of "Tick". Each "Tick" is a single count of a valid
    // qudrature transition. This means the endcoder is in 4x mode
    uint32_t getCountQuad(TimerFormat_t format)
    {
      uint32_t CNT_RegisterValue = LL_TIM_GetCounter(_timerObj.handle.Instance);
      //uint32_t Prescalerfactor = LL_TIM_GetPrescaler(_timerObj.handle.Instance) + 1; 
      uint32_t return_value;
      switch (format) {
        case TICK_FORMAT:
        default :
          return_value = CNT_RegisterValue;
          break;
      }
      return return_value;
    }

    // Returns the current state of the direction bit for the quadrature decoder
    // 0 = forward, 1 = backward (0 = up-counter, 1 = down-counter)
    bool getDirBit(){
      return (_timerObj.handle.Instance->CR1 & TIM_CR1_DIR);
    }

  private:
    //TIM_TypeDef *_instance; // Timer instance (TIM1, TIM2, etc)
    HardwareTimer *MyTim; 
    timerObj_t _timerObj;
    const uint32_t CH1Pin;
    const uint32_t CH2Pin;
    uint32_t quadCnt = 0; // The current count of the quadrature decoder

    // Sets up the basic settings of the timer hardware such 
    // as period, prescaler, etc.
    void basicTimerSetup(TIM_TypeDef *instance)
    {

      // Configure the timer object with defaults that will work
      // with the timer decoder setup
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

      _timerObj.preemptPriority = TIM_IRQ_PRIO;
      _timerObj.subPriority = TIM_IRQ_SUBPRIO;

      /* Enable timer clock. Even if it is also done in HAL_TIM_Base_MspInit(),
         it is done there so that it is possible to write registers right now */
      enableTimerClock(&(_timerObj.handle));

      // Configure timer with some default values
      _timerObj.handle.Init.Prescaler = 0;
      _timerObj.handle.Init.Period  = MAX_RELOAD;//= MAX_RELOAD;
      _timerObj.handle.Init.CounterMode = TIM_COUNTERMODE_UP;
      _timerObj.handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
      #if defined(TIM_RCR_REP)
        _timerObj.handle.Init.RepetitionCounter = 0;
      #endif
      _timerObj.handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    
      // Initialize the TIM low level resources
      HAL_TIM_Base_Init(&(_timerObj.handle));
    }

    // Sets up the setttings quadrature decoder mode of the timer such as
    // the encoder mode, the polarity, etc.
    void quadratureSetup(){
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

        // Configure the TIM in the desired functionality
        
        HAL_TIM_Encoder_Init(&(_timerObj.handle), &TIM_EncoderInitStruct); 

        // Start the Hardware Timers

        HAL_TIM_Encoder_Start(&(_timerObj.handle), TIM_CHANNEL_ALL);
        
        //HAL_TIM_Encoder_Init(MyTim->getHandle(), &TIM_EncoderInitStruct); 
        // Start the Hardware Timers
        //HAL_TIM_Encoder_Start(MyTim->getHandle(), TIM_CHANNEL_ALL);
    }

    // Setup the timer pins as inputs
    void pinInputSetup(uint32_t CH1Pin, uint32_t CH2Pin){
        // In the STM32 Arduino Core, configuring them as Inputs is sufficient for them 
        // to be used as with the Timers
        //pinMode(CH1Pin, INPUT);
        //pinMode(CH2Pin, INPUT);
    }


};

