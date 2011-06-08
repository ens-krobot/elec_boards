#include "encoder.h"
#include "motor.h"
#include <cfg/macros.h>
#include <drv/gpio_stm32.h>
#include "stm32lib/stm32f10x_tim.h"
#include "stm32lib/stm32f10x_rcc.h"
#include "stm32lib/stm32f10x.h"



void encodersInit(void)	{
	
	TIM_ICInitTypeDef  TIM_ICInitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	
	// Enable clocking on GPIOA, GPIOB, GPIOC and GPIOD 	
			RCC->APB2ENR |= RCC_APB2_GPIOA |  RCC_APB2_GPIOB | RCC_APB2_GPIOC ;
			
	/* PinOut
	Encoder 1 : A1 := PA6
				B1 := PA7
				
	Encoder 2 : A2 := PC6
				B2 := PC7
				
	Encoder 3 : A3 := PA8
				B3 := PA9
				
	Encoder 4 : A4 := PB6
				B4 := PB7*/
				
	// TIM3 (A1,B1) ; TIM8 (A2,B2) ; TIM1 (A3,B3) ; TIM4 (A4,B4)
	
	// TIMx clock enable
	
		RCC->APB1ENR |= RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4;
		RCC->APB2ENR |= RCC_APB2Periph_TIM8 | RCC_APB2Periph_TIM1;
		
	// Configuring PinOut mode
		
		stm32_gpioPinConfig(((struct stm32_gpio *)GPIOA_BASE),		
			    BV(6) | BV(7) | BV(8) | BV(9), GPIO_MODE_IN_FLOATING,                 
			    GPIO_SPEED_50MHZ);
	
		stm32_gpioPinConfig(((struct stm32_gpio *)GPIOB_BASE),		
			    BV(6) | BV(7), GPIO_MODE_IN_FLOATING,                 
			    GPIO_SPEED_50MHZ);
			    
		stm32_gpioPinConfig(((struct stm32_gpio *)GPIOC_BASE),		
			    BV(6) | BV(7), GPIO_MODE_IN_FLOATING,                 
			    GPIO_SPEED_50MHZ);
			    
	// Configuring TIMx base
	
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
  	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);
  	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
  	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	
	TIM_ICStructInit(&TIM_ICInitStructure);
  	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  	TIM_ICInitStructure.TIM_ICFilter = 0x0;
  	
  	
  	
  		// Configuring chanel 1
  			TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
  			TIM_ICInit(TIM3, &TIM_ICInitStructure);
  			TIM_ICInit(TIM8, &TIM_ICInitStructure);
  			TIM_ICInit(TIM1, &TIM_ICInitStructure);
  			TIM_ICInit(TIM4, &TIM_ICInitStructure);
  			
  		// Configuring chanel 2
  			TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
  			TIM_ICInit(TIM3, &TIM_ICInitStructure);
  			TIM_ICInit(TIM8, &TIM_ICInitStructure);
  			TIM_ICInit(TIM1, &TIM_ICInitStructure);
  			TIM_ICInit(TIM4, &TIM_ICInitStructure);
  			
  	// Réglage de la configuration de l'encodeur : quadrature ou non
  	
  		TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
  		TIM_EncoderInterfaceConfig(TIM8, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
  		TIM_EncoderInterfaceConfig(TIM1, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
  		TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
  		
  	// Enable encoders
  	
  		TIM_Cmd(TIM3, ENABLE);
  		TIM_Cmd(TIM8, ENABLE);
  		TIM_Cmd(TIM1, ENABLE);
  		TIM_Cmd(TIM4, ENABLE);
  	
  	resetEncoderCount(ENCODER1 | ENCODER2 | ENCODER3 | ENCODER4 );
	
	return;
}

// Obtentions des valeurs des compteurs
uint16_t getEncoderCount(unsigned short encoder)  {
	switch (encoder)  {
		case ENCODER1 :
			return TIM_GetCounter(TIM3);
			break;
		case ENCODER2 :
			return TIM_GetCounter(TIM8);
			break;
		case ENCODER3 :
			return TIM_GetCounter(TIM1);
			break;
		case ENCODER4 :
			return TIM_GetCounter(TIM4);
			break;	
	}
	return 0;
}

// Reset de la valeur des compteurs à O	
void resetEncoderCount(unsigned short encoders)	{
	
	if(encoders & ENCODER1)  {
		TIM_SetCounter(TIM3,0);
	}
	
	if(encoders & ENCODER2)  {
		TIM_SetCounter(TIM8,0);
	}
	
	if(encoders & ENCODER3)  {
		TIM_SetCounter(TIM1,0);
	}
	
	if(encoders & ENCODER4)  {
		TIM_SetCounter(TIM4,0);
	}
	
	return;
}

uint8_t getEncoderDirection(unsigned short encoder)  {

	uint8_t direction;
	switch (encoder)  {
		case ENCODER1 :
			if ((TIM3->CR1 & TIM_CR1_DIR)!=0)  {
				direction = BACKWARD_DIRECTION;  
				}
			else  {
				direction = FORWARD_DIRECTION;
				}
			break;
		case ENCODER2 :
			if ((TIM8->CR1 & TIM_CR1_DIR)!=0)  {
				direction = BACKWARD_DIRECTION;  
				}
			else  {
				direction = FORWARD_DIRECTION;
				}
			break;
		case ENCODER3 :
			if ((TIM1->CR1 & TIM_CR1_DIR)!=0)  {
				direction = BACKWARD_DIRECTION;  
				}
			else  {
				direction = FORWARD_DIRECTION;
				}
			break;
		case ENCODER4 :
			if ((TIM4->CR1 & TIM_CR1_DIR)!=0)  {
				direction = BACKWARD_DIRECTION;  
				}
			else  {
				direction = FORWARD_DIRECTION;
				}
			break;
	}
	return direction;
}

