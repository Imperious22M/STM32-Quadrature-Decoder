// Author: Scott Roelker
// Setup GPIO and Timer for quadrature decoder

// TIM2 and TIM3 will be used 

#include <stm32yyxx_ll_tim.h>

// Starts both quad encoders at CNT = 32768 to avoid overflows
// CNT is reset when encoders are read
void setupQuadTimers(){
    static HardwareTimer *quadA = new HardwareTimer(TIM2);

    // TIM 2 is 32 bit
    // Configure Quad Encoder A (TIM2) ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
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
    TIM2->CNT = 0x8000;  //reset the counter before we use it 

    // TIM3 is 16 bit
    // Configure Quad Encoder B (TIM3) ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // configure GPIO PB4 & PB5 as inputs for Encoder
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;  // Enable clock for GPIOB
 
    GPIOB->MODER   |= GPIO_MODER_MODER4_1 | GPIO_MODER_MODER5_1 ;           //PB4 & PB5 as Alternate Function   /*!< GPIO port mode register,               Address offset: 0x00      */
    GPIOB->OTYPER  |= GPIO_OTYPER_OT_4 | GPIO_OTYPER_OT_5 ;                 //PB4 & PB5 as Inputs               /*!< GPIO port output type register,        Address offset: 0x04      */
    GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR4 | GPIO_OSPEEDER_OSPEEDR5;      //0x11000000;//                        /*!< GPIO port output speed register,       Address offset: 0x08      */
    GPIOB->PUPDR   |= GPIO_PUPDR_PUPDR4_1 | GPIO_PUPDR_PUPDR5_1 ;           // Pull Down                        /*!< GPIO port pull-up/pull-down register,  Address offset: 0x0C      */
    GPIOB->AFR[0]  |= GPIO_AFRL_AFRL5_1 | GPIO_AFRL_AFRL4_1;//0x11000011 ;                                          //  AF01 for PB4 & PB5              /*!< GPIO alternate function registers,     Address offset: 0x20-0x24 */
    GPIOB->AFR[1]  |= 0x00000000 ;                                          //                                  /*!< GPIO alternate function registers,     Address offset: 0x20-0x24 */
   
    // configure TIM3 as Encoder input
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;  // Enable clock for TIM3
 
    TIM3->CR1   = 0x0001;     // CEN(Counter ENable)='1'     < TIM control register 1
    TIM3->SMCR  = 0x0003;     // SMS='011' (Encoder mode 3)  < TIM slave mode control register
    TIM3->CCMR1 = 0x0101;     // CC1S='01' CC2S='01'         < TIM capture/compare mode register 1
    TIM3->CCMR2 = 0x0000;     //                             < TIM capture/compare mode register 2
    TIM3->CCER  = 0x0011;     // CC1P CC2P                   < TIM capture/compare enable register
    TIM3->PSC   = 0x0000;     // Prescaler = (0+1)           < TIM prescaler
    TIM3->ARR   = 0xffffffff; // reload at 0xfffffff         < TIM auto-reload register
    //Tim3->BDTR
    TIM3->CNT = 0x8000;  //reset the counter before we use it 
}
// read functions reset actual quadrature cnt register
// to 32768 (base 10) to avoid overflows during normal operation
uint32_t readQuadA(){
    uint32_t ret = (uint16_t)TIM2->CNT; // Treat the encoder as a 16-bit timer (even if TIM2 is a 32 bit timer)
    TIM2->CNT = 0x8000;
    return ret;
}
uint32_t readQuadB(){
    uint32_t ret = TIM3->CNT;
    TIM3->CNT = 0x8000;
    return ret;
}
bool readDirQuadA(){
    return TIM2->CR1 & TIM_CR1_DIR; // DOUBLE CHECK!!
}