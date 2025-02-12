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

// Create an QuadratureDecoder object, this wraps around the functionality of the decoder
// The Timer used MUST support Quadrature decoding, otherwise the functions will not work
QuadratureDecoder quadDecoder(TIM1, T1CH1_Pin, T1CH2_Pin);

void setup()
{
  Serial.begin(9600);

}

void loop()
{
  Serial.println(quadDecoder.getCountQuad(TICK_FORMAT));
  Serial.println("~~~~");
  delay(1000);
}