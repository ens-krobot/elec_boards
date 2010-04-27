/*
 * Wrapper to use the Encoder interface
 * Xavier Lagorce
 */

#include "encoder.h"

/*
 * Function to initialise one encoder interface
 */
void encodersInit(void) {

  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_ICInitTypeDef       TIM_ICInitStructure;
  GPIO_InitTypeDef        GPIO_InitStructure;
  
  //Enable GPIOA and GPIOB clock
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  
  //Enable timer clock
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
  
  //Setup timer for quadrature encoder interface
  //Encoder1 A channel at PA0.6 (Ch1)
  //         B channel at PA0.7 (Ch2)
  //Encoder2 A channel at PA0.8 (Ch1)
  //         B channel at PA0.9 (Ch2)
  //Encoder3 A channel at PB0.6 (Ch1)
  //         B channel at PB0.7 (Ch2)
  GPIO_InitStructure.GPIO_Pin     = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed   = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin     = GPIO_Pin_6|GPIO_Pin_7;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  // TimeBase configuration
  TIM_TimeBaseStructure.TIM_Prescaler     = 0;
  TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_Period        = 0xFFFF;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
  
  //Initialize input capture structure: Ch1
  TIM_ICStructInit(&TIM_ICInitStructure);
  TIM_ICInitStructure.TIM_Channel     = TIM_Channel_1;
  TIM_ICInitStructure.TIM_ICPolarity  = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter    = 0;
  TIM_ICInit(TIM1, &TIM_ICInitStructure);
  TIM_ICInit(TIM3, &TIM_ICInitStructure);
  TIM_ICInit(TIM4, &TIM_ICInitStructure);
  
  //Initialize input capture structure: Ch2
  TIM_ICInitStructure.TIM_Channel     = TIM_Channel_2;
  TIM_ICInit(TIM1, &TIM_ICInitStructure);   
  TIM_ICInit(TIM3, &TIM_ICInitStructure);   
  TIM_ICInit(TIM4, &TIM_ICInitStructure);   
  
  //Encoder Interface Configuration
  TIM_EncoderInterfaceConfig(TIM1,
                             TIM_EncoderMode_TI12,
                             TIM_ICPolarity_Rising,
                             TIM_ICPolarity_Rising);
  TIM_EncoderInterfaceConfig(TIM3,
                             TIM_EncoderMode_TI12,
                             TIM_ICPolarity_Rising,
                             TIM_ICPolarity_Rising);
  TIM_EncoderInterfaceConfig(TIM4,
                             TIM_EncoderMode_TI12,
                             TIM_ICPolarity_Rising,
                             TIM_ICPolarity_Rising);
  //Enable timer Peripherals
  TIM_Cmd(TIM1,ENABLE);
  TIM_Cmd(TIM3,ENABLE);
  TIM_Cmd(TIM4,ENABLE);
}

/*
 * Helper to get the current encoder position
 *  encoder : get the value form this encoder
 */
uint16_t getEncoderPosition(uint8_t encoder) {
  switch(encoder) {
    case ENCODER1:
      return TIM_GetCounter(TIM1);
      break;
    case ENCODER2:
      return TIM_GetCounter(TIM3);
      break;
    case ENCODER3:
      return TIM_GetCounter(TIM4);
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
      TIM_SetCounter(TIM1, 0);
      break;
    case ENCODER2:
      TIM_SetCounter(TIM3, 0);
      break;
    case ENCODER3:
      TIM_SetCounter(TIM4, 0);
      break;
    default:
      break;
  }
}
