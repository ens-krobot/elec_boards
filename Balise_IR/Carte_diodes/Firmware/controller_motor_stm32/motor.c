#include "motor.h"
#include <cfg/macros.h>
#include <drv/gpio_stm32.h>
#include "stm32lib/stm32f10x_tim.h"

/*Private variables */

	TIM_OCInitTypeDef  TIM_OCInitStructure;

	unsigned short def_motors[4]={MOTEUR1, MOTEUR2, MOTEUR3, MOTEUR4};
	motor_status motors_status[4];

void motorsInit(void) {

	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	
	uint32_t SystemClock = 72000000;
	uint32_t Tim2DesiredClock = 72000000;
	uint16_t PrescalerValue = 0;
	
	int i;
	
	// Enable clocking on GPIOA, GPIOB, GPIOC and GPIOD 	
			RCC->APB2ENR |= RCC_APB2_GPIOA |  RCC_APB2_GPIOB | RCC_APB2_GPIOC | RCC_APB2_GPIOD;
			
	// Enable clocking on TIM2
			RCC->APB1ENR |= RCC_APB1_TIM2;
	//Def I/O of motors
		//Port A
			stm32_gpioPinConfig(((struct stm32_gpio *)GPIOA_BASE),		
			    BV(0) | BV(1) | BV(2) | BV(3), GPIO_MODE_AF_PP,                 
			    GPIO_SPEED_50MHZ); //TIM2 on PWM Pins
			stm32_gpioPinConfig(((struct stm32_gpio *)GPIOA_BASE),		
			    BV(5) | BV(10) , GPIO_MODE_OUT_PP,                 
			    GPIO_SPEED_50MHZ);
			    
		//Port B
			stm32_gpioPinConfig(((struct stm32_gpio *)GPIOB_BASE),		
			    BV(1) | BV(14) | BV(15) | BV(5) | BV(9), GPIO_MODE_OUT_PP,                 
			    GPIO_SPEED_50MHZ);
			    
		//Port C
			stm32_gpioPinConfig(((struct stm32_gpio *)GPIOC_BASE),		
			    BV(4) | BV(5) | BV(10) | BV(11) | BV(9), GPIO_MODE_OUT_PP,                 
			    GPIO_SPEED_50MHZ);
			    
		//Port D
			stm32_gpioPinConfig(((struct stm32_gpio *)GPIOD_BASE),		
			    BV(2), GPIO_MODE_OUT_PP,                 
			    GPIO_SPEED_50MHZ);
			    
	//Init of TIM2
	  	
	  	
	  	/*Calcul du Prescalar */
	  	PrescalerValue = (uint16_t) (SystemClock/Tim2DesiredClock)-1;
	  	//PrescalerValue = 0;
	  	
	  	/*Config du timer2 */
	  	TIM_TimeBaseStructure.TIM_Period = 3600; //Horloge : 20kHz
	  	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
	  	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	  	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	  	
	  	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	  	
	  	/*Initialisation des PWMs*/
	  		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
			TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
			TIM_OCInitStructure.TIM_Pulse = 0;
			TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

		/*Chanel 1*/
		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	  	TIM_OC1Init(TIM2, &TIM_OCInitStructure);
	  	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
	  	
	  	/*Chanel 2*/
		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	  	TIM_OC2Init(TIM2, &TIM_OCInitStructure);
	  	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
	  	
	  	/*Chanel 3*/
		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	  	TIM_OC3Init(TIM2, &TIM_OCInitStructure);
	  	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
	  	
	  	/*Chanel 4*/
		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	  	TIM_OC4Init(TIM2, &TIM_OCInitStructure);
	  	TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);
	  	
	  	TIM_ARRPreloadConfig(TIM2, ENABLE);
	  	
	  	/* TIM2 enable counter */
		TIM_Cmd(TIM2, ENABLE);
	  	
	/* Init motors_status*/
	for(i=0;i<NUM_MOTORS;i++)
		{
			motors_status[i].currentPWM=0;
			motors_status[i].desiredPosition=0;
			motors_status[i].currentPosition=0;
			motors_status[i].direction=0;
			motors_status[i].speed=0;
		}
		
  	return;
}

void setMotors(unsigned short motors, unsigned short direction) {
	/*Def le sens de rotation des moteurs : 0 forward
											1 backward */
	if( motors & MOTEUR1 )
		{
			switch (direction)	{
				case FORWARD_DIRECTION :
						stm32_gpioPinWrite(((struct stm32_gpio *)GPIOC_BASE), BV(4), 1);
						stm32_gpioPinWrite(((struct stm32_gpio *)GPIOC_BASE), BV(5), 0);
						break;
				case BACKWARD_DIRECTION :
						stm32_gpioPinWrite(((struct stm32_gpio *)GPIOC_BASE), BV(4), 0);
						stm32_gpioPinWrite(((struct stm32_gpio *)GPIOC_BASE), BV(5), 1);
						break;
			}
		}
		
	if( motors & MOTEUR2 )
		{
			switch (direction)	{
				case FORWARD_DIRECTION :
						stm32_gpioPinWrite(((struct stm32_gpio *)GPIOB_BASE), BV(1), 1);
						stm32_gpioPinWrite(((struct stm32_gpio *)GPIOB_BASE), BV(14), 0);
						break;
				case BACKWARD_DIRECTION :
						stm32_gpioPinWrite(((struct stm32_gpio *)GPIOB_BASE), BV(1), 0);
						stm32_gpioPinWrite(((struct stm32_gpio *)GPIOB_BASE), BV(14), 1);
						break;
			}
		}
		
	if( motors & MOTEUR3 )
		{
			switch (direction)	{
				case FORWARD_DIRECTION :
						stm32_gpioPinWrite(((struct stm32_gpio *)GPIOC_BASE), BV(10), 1);
						stm32_gpioPinWrite(((struct stm32_gpio *)GPIOC_BASE), BV(11), 0);
						break;
				case BACKWARD_DIRECTION :
						stm32_gpioPinWrite(((struct stm32_gpio *)GPIOC_BASE), BV(10), 0);
						stm32_gpioPinWrite(((struct stm32_gpio *)GPIOC_BASE), BV(11), 1);
						break;
			}
		}
		
	if( motors & MOTEUR4 )
		{
			switch (direction)	{
				case FORWARD_DIRECTION :
						stm32_gpioPinWrite(((struct stm32_gpio *)GPIOB_BASE), BV(5), 1);
						stm32_gpioPinWrite(((struct stm32_gpio *)GPIOB_BASE), BV(9), 0);
						break;
				case BACKWARD_DIRECTION :
						stm32_gpioPinWrite(((struct stm32_gpio *)GPIOB_BASE), BV(5), 0);
						stm32_gpioPinWrite(((struct stm32_gpio *)GPIOB_BASE), BV(9), 1);
						break;
			}
		}
	return;
}

void enableMotors(unsigned short motors) {
	/*Active le pont H du MOTEURX */
	if( motors & MOTEUR1 )
		{
			stm32_gpioPinWrite(((struct stm32_gpio *)GPIOA_BASE), BV(5), 1);
		}
		
	if( motors & MOTEUR2 )
		{
			stm32_gpioPinWrite(((struct stm32_gpio *)GPIOB_BASE), BV(15), 1);
		}
		
	if( motors & MOTEUR3 )
		{
			stm32_gpioPinWrite(((struct stm32_gpio *)GPIOA_BASE), BV(10), 1);
		}
		
	if( motors & MOTEUR4 )
		{
			stm32_gpioPinWrite(((struct stm32_gpio *)GPIOD_BASE), BV(2), 1);
		}
	return;
}

void disableMotors(unsigned short motors) {
	/*Active le pont H du MOTEURX */
	if( motors & MOTEUR1 )
		{
			stm32_gpioPinWrite(((struct stm32_gpio *)GPIOA_BASE), BV(5), 0);
		}
		
	if( motors & MOTEUR2 )
		{
			stm32_gpioPinWrite(((struct stm32_gpio *)GPIOB_BASE), BV(15), 0);
		}
		
	if( motors & MOTEUR3 )
		{
			stm32_gpioPinWrite(((struct stm32_gpio *)GPIOA_BASE), BV(10), 0);
		}
		
	if( motors & MOTEUR4 )
		{
			stm32_gpioPinWrite(((struct stm32_gpio *)GPIOD_BASE), BV(2), 0);
		}
	return;
}

void setVelocity(unsigned short motors, int32_t velocity)  {
	
	if(velocity > MAX_PWM) velocity=MAX_PWM;
	if(velocity < -MAX_PWM) velocity=-MAX_PWM;
		
	if( motors & MOTEUR1 )
		{
			//stm32_gpioPinWrite(((struct stm32_gpio *)GPIOA_BASE), BV(0), 1);
			if( velocity >= 0) {
				setMotors(MOTEUR1, FORWARD_DIRECTION);
			}
	  		else {
	  			setMotors(MOTEUR1, BACKWARD_DIRECTION);
	  			velocity = -velocity;
	  		}
	  		if(velocity==MAX_PWM) LED1_ON();
	  		else LED1_OFF();
			TIM_OCInitStructure.TIM_Pulse = (uint16_t)velocity;
			TIM_OC1Init(TIM2, &TIM_OCInitStructure);
	  		TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
	  		
		}
		
	if( motors & MOTEUR2 )
		{
			//stm32_gpioPinWrite(((struct stm32_gpio *)GPIOA_BASE), BV(1), 1);
			if( velocity >= 0) {
				setMotors(MOTEUR2, FORWARD_DIRECTION);
			}
	  		else {
	  			setMotors(MOTEUR2, BACKWARD_DIRECTION);
	  			velocity = -velocity;
	  		}
	  		//if(velocity==MAX_PWM) LED2_ON();
	  		//else LED2_OFF();
			TIM_OCInitStructure.TIM_Pulse = (uint16_t)velocity;
			TIM_OC2Init(TIM2, &TIM_OCInitStructure);
	  		TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
		}
		
	if( motors & MOTEUR3 )
		{
			//stm32_gpioPinWrite(((struct stm32_gpio *)GPIOA_BASE), BV(2), 1);
			if( velocity >= 0) {
				setMotors(MOTEUR3, FORWARD_DIRECTION);
			}
	  		else {
	  			setMotors(MOTEUR3, BACKWARD_DIRECTION);
	  			velocity = -velocity;
	  		}
	  		if(velocity==MAX_PWM) LED3_ON();
	  		else LED3_OFF();
			TIM_OCInitStructure.TIM_Pulse = (uint16_t)velocity;
			TIM_OC3Init(TIM2, &TIM_OCInitStructure);
	  		TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
		}
		
	if( motors & MOTEUR4 )
		{
			//stm32_gpioPinWrite(((struct stm32_gpio *)GPIOA_BASE), BV(3), 1);
			if( velocity >= 0) {
				setMotors(MOTEUR4, FORWARD_DIRECTION);
			}
	  		else {
	  			setMotors(MOTEUR4, BACKWARD_DIRECTION);
	  			velocity = -velocity;
	  		}
	  		if(velocity==MAX_PWM) LED4_ON();
	  		else LED4_OFF();
			TIM_OCInitStructure.TIM_Pulse = (uint16_t)velocity;
			TIM_OC4Init(TIM2, &TIM_OCInitStructure);
	  		TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);
		}
		
	TIM_ARRPreloadConfig(TIM2, ENABLE);
	return;
}

void breakMotors(unsigned short motors)  {
	if( motors & MOTEUR1 )
		{
			stm32_gpioPinWrite(((struct stm32_gpio *)GPIOC_BASE), BV(4), 1);
			stm32_gpioPinWrite(((struct stm32_gpio *)GPIOC_BASE), BV(5), 1);
		}
		
	if( motors & MOTEUR2 )
		{
			stm32_gpioPinWrite(((struct stm32_gpio *)GPIOB_BASE), BV(1), 1);
			stm32_gpioPinWrite(((struct stm32_gpio *)GPIOB_BASE), BV(14), 1);
		}
		
	if( motors & MOTEUR3 )
		{
			stm32_gpioPinWrite(((struct stm32_gpio *)GPIOC_BASE), BV(10), 1);
			stm32_gpioPinWrite(((struct stm32_gpio *)GPIOC_BASE), BV(11), 1);
		}
		
	if( motors & MOTEUR4 )
		{
			stm32_gpioPinWrite(((struct stm32_gpio *)GPIOB_BASE), BV(5), 1);
			stm32_gpioPinWrite(((struct stm32_gpio *)GPIOB_BASE), BV(9), 1);
		}
	return;
}

void freeMotors(unsigned short motors)  {
	if( motors & MOTEUR1 )
		{
			stm32_gpioPinWrite(((struct stm32_gpio *)GPIOC_BASE), BV(4), 0);
			stm32_gpioPinWrite(((struct stm32_gpio *)GPIOC_BASE), BV(5), 0);
		}
		
	if( motors & MOTEUR2 )
		{
			stm32_gpioPinWrite(((struct stm32_gpio *)GPIOB_BASE), BV(1), 0);
			stm32_gpioPinWrite(((struct stm32_gpio *)GPIOB_BASE), BV(14), 0);
		}
		
	if( motors & MOTEUR3 )
		{
			stm32_gpioPinWrite(((struct stm32_gpio *)GPIOC_BASE), BV(10), 0);
			stm32_gpioPinWrite(((struct stm32_gpio *)GPIOC_BASE), BV(11), 0);
		}
		
	if( motors & MOTEUR4 )
		{
			stm32_gpioPinWrite(((struct stm32_gpio *)GPIOB_BASE), BV(5), 0);
			stm32_gpioPinWrite(((struct stm32_gpio *)GPIOB_BASE), BV(9), 0);
		}
	return;
}
