#include "emission.h"

#include <cfg/macros.h>
#include <drv/gpio_stm32.h>
#include "stm32lib/stm32f10x_tim.h"
#include "stm32lib/stm32f10x_rcc.h"
#include <math.h>

/*Private variables */

	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
PROC_DEFINE_STACK(stack_emission, KERN_MINSTACKSIZE * 2);
int currentLED;

void emissionInit()  {

	/*
	-------------------------------------
	Def des pinouts
	-------------------------------------
	PC9 -> Clock 56 kHz
	PC8 -> SDI : registre à décalage
	PC7 -> CLK : clock assurant un décalage sur FM
	PC6 -> Latch : copying the state register
	
	PA8 -> Desired signal information
	*/
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	
	uint32_t SystemClock = 72000000;
	uint32_t TIM8DesiredClock = 72000000;
	uint16_t PrescalerValue = 0;
	
	// Enable clocking on GPIOA, GPIOB, GPIOC and GPIOD 	
			RCC->APB2ENR |= RCC_APB2_GPIOA |  RCC_APB2_GPIOB | RCC_APB2_GPIOC | RCC_APB2_GPIOD;
			
	// Enable clocking on TIM8
			RCC->APB2ENR |= RCC_APB2Periph_TIM8; // TIM8 configuring on APB2
			
	//Def I/O of motors
		//Port A
			stm32_gpioPinConfig(((struct stm32_gpio *)GPIOA_BASE),		
			    BV(8), GPIO_MODE_OUT_PP,                 
			    GPIO_SPEED_50MHZ); 
			    
			    
		//Port C
			stm32_gpioPinConfig(((struct stm32_gpio *)GPIOC_BASE),		
			    BV(9), GPIO_MODE_AF_PP,                 
			    GPIO_SPEED_50MHZ);
			    
			stm32_gpioPinConfig(((struct stm32_gpio *)GPIOC_BASE),		
			    BV(6) | BV(7) | BV(8), GPIO_MODE_OUT_PP,                 
			    GPIO_SPEED_50MHZ);
			    
		/*//Port D
			stm32_gpioPinConfig(((struct stm32_gpio *)GPIOD_BASE),		
			    BV(2), GPIO_MODE_OUT_PP,                 
			    GPIO_SPEED_50MHZ);*/
			    
	//Init of TIM8
	  	
	  	
	  	/*Calcul du Prescalar */
	  	PrescalerValue = (uint16_t) (SystemClock/TIM8DesiredClock)-1;
	  	//PrescalerValue = 0;
	  	
	  	/*Config du timer8 */
	  	TIM_TimeBaseStructure.TIM_Period = PWM_56k; //Horloge : 20kHz
	  	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
	  	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	  	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	  	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);
	  	
	  	/*Initialisation des PWMs*/
	  		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
			TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
			TIM_OCInitStructure.TIM_Pulse = 0;
			TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	  	
	  	/*Chanel 4*/
		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	  	TIM_OC4Init(TIM8, &TIM_OCInitStructure);
	  	TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable);
	  	TIM_ARRPreloadConfig(TIM8, ENABLE);
		TIM_CtrlPWMOutputs(TIM8, ENABLE);	//only for TIM1 and TIM8
	  	
	  	/* TIM8 enable counter */
		TIM_Cmd(TIM8, ENABLE); 

		
		
		/*Active 56k Clock*/
		enableClock();
		
		/* Creation of a new process*/
		proc_new(emission_process, NULL, sizeof(stack_emission), stack_emission);

		// Initialize global variable
		currentLED=0;
		
	return;
}

void enableClock(void)  {

	TIM_OCInitStructure.TIM_Pulse = (uint16_t)(PWM_56k/2);
	TIM_OC4Init(TIM8, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable);
	//stm32_gpioPinWrite(((struct stm32_gpio *)GPIOC_BASE), BV(9), 1);
	return;
}

void disableClock(void)  {

	TIM_OCInitStructure.TIM_Pulse = (uint16_t)(0);
	TIM_OC4Init(TIM8, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable);
	
	return;
}

void writeSDI(unsigned short value)  {
	if(value == 0) {
		stm32_gpioPinWrite(((struct stm32_gpio *)GPIOC_BASE), BV(8), 0);
	}
	else {
		stm32_gpioPinWrite(((struct stm32_gpio *)GPIOC_BASE), BV(8), 1);
	}
	wait_clock(CYCLE_WAIT);
	return;
}

void writeCLK(unsigned short value)  {
	if(value == 0) {
		stm32_gpioPinWrite(((struct stm32_gpio *)GPIOC_BASE), BV(7), 0);
	}
	else {
		stm32_gpioPinWrite(((struct stm32_gpio *)GPIOC_BASE), BV(7), 1);
	}
	wait_clock(CYCLE_WAIT);
	return;
}

void writeLatch(unsigned short value)  {
	if(value == 0) {
		stm32_gpioPinWrite(((struct stm32_gpio *)GPIOC_BASE), BV(6), 0);
	}
	else {
		stm32_gpioPinWrite(((struct stm32_gpio *)GPIOC_BASE), BV(6), 1);
	}
	wait_clock(CYCLE_WAIT);
	return;
}

void writeCommande(unsigned short value)  {
	if(value == 0) {
		stm32_gpioPinWrite(((struct stm32_gpio *)GPIOA_BASE), BV(8), 0);
	}
	else {
		stm32_gpioPinWrite(((struct stm32_gpio *)GPIOA_BASE), BV(8), 1);
	}
	wait_clock(CYCLE_WAIT);
	return;
}

void NORETURN emission_process(void)
{
  Timer timer_emission_process;
  int i,j;
  unsigned short word[LENGTH_WORD]={1};
  unsigned short state=0;
  int emission_time=EMISSION_TIME_HIGH;
  
  
  for(i=0;i<LENGTH_WORD;i++)  {
  	word[i]=1;
  
  }
  
  initLatch();

//sélection de la diode de départ
  writeCommande(0);

  		writeSDI(1);
		wait_clock(100);

  		writeCLK(1);
  		wait_clock(100);
  		writeCLK(0);
		wait_clock(100);

  		writeLatch(1);
  		wait_clock(100);
  		writeLatch(0);
  		wait_clock(100);
  		
  i=0;
  j=0;
  while(1) 
	{
	
		if (state == 0) emission_time = EMISSION_TIME_HIGH;
		if (state == 1) emission_time = EMISSION_TIME_LOW;

		timer_setDelay(&timer_emission_process, us_to_ticks((utime_t)(emission_time)));
	  	timer_setEvent(&timer_emission_process);
	  	timer_add(&timer_emission_process);// Start process timer

		if (state == 0 && word[j] == 1) writeCommande(1);
		else writeCommande(0);

	      	if(state == 1)
		{
			state=0;
			j++;
			if(j==LENGTH_WORD)
			{
				j=0;
				i++;
				if(i==NUM_LEDS) 
				{
					i=0;
  					writeSDI(1);
				}
				else writeSDI(0);

				wait_clock(100);
			  	writeCLK(1);
				wait_clock(100);
			  	writeCLK(0);
				wait_clock(100);

				writeCommande(0);

				wait_clock(100);
				writeLatch(1);
		  		wait_clock(100);
		  		writeLatch(0);
		  		wait_clock(100);
			}
		}
		else state=1;

  	timer_waitEvent(&timer_emission_process); // Wait until the end of counting
  }
}

void initLatch(void)  {
	int i=0;
	writeCommande(0);
	for(i=0; i<NUM_LEDS;i++)  {
  		writeSDI(0);
  		
		//wait_clock(100);

  		writeCLK(1);
  		//wait_clock(100);
  		writeCLK(0);
		//wait_clock(100);

  		writeLatch(1);
  		//wait_clock(100);
  		writeLatch(0);
  		//wait_clock(100);
	}
	return;
}

void wait_clock(int nb_clk)
{
	int i=0;
	for (i=0 ; i<nb_clk ; i++);
}

void selectLed(int num_led)  {
  int i;
  writeSDI(1); // and we wait ...
  writeCLK(1); // on the rising edge  
  writeCLK(0);
  writeSDI(0);
  for(i=0; i<num_led;i++)  { // and we go
    writeCLK(1);
    writeCLK(0);
    }
  // Finally, writting Latch
  writeLatch(1);
  writeLatch(0);
  return;
}

void nextLed(void)  {
  if(currentLED==0)  { //Start of the cycle
    writeSDI(1);
    writeCLK(1);
    writeCLK(0);
    writeSDI(0);
  }
  else { //only a rising on clock
    writeCLK(1);
    writeCLK(0);
  }
  writeLatch(1);
  writeLatch(0);
  currentLED++;
  currentLED = (currentLED==NUM_LEDS) ? 0: currentLED;
  return;
}

void selectDoubleLed(int num_first_led) {
  int selectLED;
  int i;
  selectLED = (num_first_led>=NUM_LEDS/2) ? num_first_led - NUM_LEDS/2 : num_first_led;
  writeSDI(1); // and we wait ...
  writeCLK(1); // on the rising edge  
  writeCLK(0);
  writeSDI(0);
  for(i=0; i<NUM_LEDS;i++) { // and we go
    writeCLK(1);
    writeCLK(0);
    }
  writeSDI(1); // and we wait again...
  writeCLK(1); // on the rising edge  
  writeCLK(0);
  writeSDI(0);
  for(i=0; i<selectLED;i++) { // and we go
    writeCLK(1);
    writeCLK(0);
    }
  // Finally, writting Latch
  writeLatch(1);
  writeLatch(0);
  return;
}
