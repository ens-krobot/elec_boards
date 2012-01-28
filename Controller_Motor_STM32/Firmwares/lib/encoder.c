/*
 * Wrapper to use the Encoder interface
 * Xavier Lagorce
 */

#include "encoder.h"

#include <cfg/macros.h>
#include <drv/gpio_stm32.h>

#include "stm32lib/stm32f10x.h"
#include "stm32lib/stm32f10x_rcc.h"
#include "stm32lib/stm32f10x_tim.h"

typedef struct {
  uint16_t origin;
  float scale;
} enc_params_t;

enc_params_t enc_params[4];

/*
 * Function to initialise one encoder interface
 */
void encodersInit(void) {

  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_ICInitTypeDef       TIM_ICInitStructure;

  //Enable GPIOA and GPIOB clock
  RCC->APB2ENR |= RCC_APB2_GPIOA | RCC_APB2_GPIOB | RCC_APB2_GPIOC;

  //Enable timer clock
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

  //Setup timer for quadrature encoder interface
  //Encoder1 A channel at PA6 (Ch1)
  //         B channel at PA7 (Ch2) -> TIM3
  //Encoder2 A channel at PC6 (Ch1)
  //         B channel at PC7 (Ch2) -> TIM8
  //Encoder3 A channel at PA8 (Ch1)
  //         B channel at PA9 (Ch2) -> TIM1
  //Encoder4 A channel at PB6 (Ch1)
  //         B channel at PB7 (Ch2) -> TIM4
  stm32_gpioPinConfig(((struct stm32_gpio *)GPIOA_BASE),
                      BV(6) | BV(7) | BV(8) | BV(9),
                      GPIO_MODE_IN_FLOATING, GPIO_SPEED_50MHZ);
  stm32_gpioPinConfig(((struct stm32_gpio *)GPIOB_BASE),
                      BV(6) | BV(7),
                      GPIO_MODE_IN_FLOATING, GPIO_SPEED_50MHZ);
  stm32_gpioPinConfig(((struct stm32_gpio *)GPIOC_BASE),
                      BV(6) | BV(7),
                      GPIO_MODE_IN_FLOATING, GPIO_SPEED_50MHZ);

  // TimeBase configuration
  TIM_TimeBaseStructure.TIM_Prescaler     = 0;
  TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_Period        = 0xFFFF;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
  TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

  //Initialize input capture structure: Ch1
  TIM_ICStructInit(&TIM_ICInitStructure);
  TIM_ICInitStructure.TIM_Channel     = TIM_Channel_1;
  TIM_ICInitStructure.TIM_ICPolarity  = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter    = 0;
  TIM_ICInit(TIM3, &TIM_ICInitStructure);
  TIM_ICInit(TIM8, &TIM_ICInitStructure);
  TIM_ICInit(TIM1, &TIM_ICInitStructure);
  TIM_ICInit(TIM4, &TIM_ICInitStructure);

  //Initialize input capture structure: Ch2
  TIM_ICInitStructure.TIM_Channel     = TIM_Channel_2;
  TIM_ICInit(TIM3, &TIM_ICInitStructure);
  TIM_ICInit(TIM8, &TIM_ICInitStructure);
  TIM_ICInit(TIM1, &TIM_ICInitStructure);
  TIM_ICInit(TIM4, &TIM_ICInitStructure);

  //Encoder Interface Configuration
  TIM_EncoderInterfaceConfig(TIM3,
                             TIM_EncoderMode_TI12,
                             TIM_ICPolarity_Rising,
                             TIM_ICPolarity_Rising);
  TIM_EncoderInterfaceConfig(TIM8,
                             TIM_EncoderMode_TI12,
                             TIM_ICPolarity_Rising,
                             TIM_ICPolarity_Rising);
  TIM_EncoderInterfaceConfig(TIM1,
                             TIM_EncoderMode_TI12,
                             TIM_ICPolarity_Rising,
                             TIM_ICPolarity_Rising);
  TIM_EncoderInterfaceConfig(TIM4,
                             TIM_EncoderMode_TI12,
                             TIM_ICPolarity_Rising,
                             TIM_ICPolarity_Rising);
  //Enable timer Peripherals
  TIM_Cmd(TIM3,ENABLE);
  TIM_Cmd(TIM8,ENABLE);
  TIM_Cmd(TIM1,ENABLE);
  TIM_Cmd(TIM4,ENABLE);

  // Initialize scaling factors
  for (int i=0; i < 4; i++) {
    enc_params[i].origin = 0;
    enc_params[i].scale= 1.0;
  }
}

/*
 * Helper to set the sacling factor for the readout helper
 */
void setEncoderScaling(uint8_t encoder, uint16_t origin, float scale) {
  enc_params[encoder].origin = origin;
  enc_params[encoder].scale = scale;
}

/*
 * Helper to get the current encoder position
 *  encoder : get the value form this encoder
 */
uint16_t getEncoderPosition(uint8_t encoder) {
  switch(encoder) {
    case ENCODER1:
      return TIM_GetCounter(TIM3);
      break;
    case ENCODER2:
      return TIM_GetCounter(TIM8);
      break;
    case ENCODER3:
      return TIM_GetCounter(TIM1);
      break;
    case ENCODER4:
      return TIM_GetCounter(TIM4);
      break;
    default:
      return 0;
  }
}
/*
 * Helper to get the current encoder position after the applying
 * some scaling factors
 */
float getEncoderPosition_f(uint8_t encoder) {
  return enc_params[encoder].scale *
    (getEncoderPosition(encoder) - enc_params[encoder].origin);
}

/*
 * Helper to get the current direction of evolution of the counter
 */
uint8_t getEncoderDirection(uint8_t encoder) {
  switch(encoder) {
    case ENCODER1:
      return ((TIM3->CR1 & TIM_CR1_DIR) != 0 ? ENCODER_DIR_DOWN : ENCODER_DIR_UP);
      break;
    case ENCODER2:
      return ((TIM8->CR1 & TIM_CR1_DIR) != 0 ? ENCODER_DIR_DOWN : ENCODER_DIR_UP);
      break;
    case ENCODER3:
      return ((TIM1->CR1 & TIM_CR1_DIR) != 0 ? ENCODER_DIR_DOWN : ENCODER_DIR_UP);
      break;
    case ENCODER4:
      return ((TIM4->CR1 & TIM_CR1_DIR) != 0 ? ENCODER_DIR_DOWN : ENCODER_DIR_UP);
      break;
    default:
      return 0;
  }
}

/*
 * Helper to reset the current encoder position
 *  encoder : reset the value from this encoder
 *
 */
void resetEncoderPosition(uint8_t encoder) {
  switch(encoder) {
    case ENCODER1:
      TIM_SetCounter(TIM3, 0);
      break;
    case ENCODER2:
      TIM_SetCounter(TIM8, 0);
      break;
    case ENCODER3:
      TIM_SetCounter(TIM1, 0);
      break;
    case ENCODER4:
      TIM_SetCounter(TIM4, 0);
      break;
    default:
      break;
  }
}
