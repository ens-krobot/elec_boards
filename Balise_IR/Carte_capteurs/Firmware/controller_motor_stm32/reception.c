#include "reception.h"

#include <cfg/macros.h>
#include <drv/gpio_stm32.h>
#include "stm32lib/stm32f10x_tim.h"
#include <math.h>

/*Private variables */

	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
PROC_DEFINE_STACK(stack_reception, KERN_MINSTACKSIZE * 2);

//unsigned short 

void receptionInit()  {

	/*
	-------------------------------------
	Def des pinouts
	-------------------------------------
	0 : PB9
	1 : PB8
	2 : PB7
	3 : PB6
	4 : PD2
	5 : PC12
	6 : PC11
	7 : PC10
	8 : PC9
	9 : PC8
	10 : PC7
	11 : PC6
	12 : PB15
	13 : PB14
	14 : PB13
	15 : PB12
	
	*/
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	
	uint32_t SystemClock = 72000000;
	uint32_t Tim3DesiredClock = 72000000;
	uint16_t PrescalerValue = 0;

	int i;
	// Enable clocking on GPIOA, GPIOB, GPIOC and GPIOD 	
			RCC->APB2ENR |= RCC_APB2_GPIOA |  RCC_APB2_GPIOB | RCC_APB2_GPIOC | RCC_APB2_GPIOD;
			
			
	//Def I/O of motors
		//Port B
			stm32_gpioPinConfig(((struct stm32_gpio *)GPIOB_BASE),		
			    BV(15) | BV(14) | BV(13) | BV(12) | BV(9) | BV(8) | BV(7) | BV(6), GPIO_MODE_IN_FLOATING,                 
			    GPIO_SPEED_50MHZ); 
			    
			    
		//Port C
			stm32_gpioPinConfig(((struct stm32_gpio *)GPIOC_BASE),		
			    BV(12) | BV(11) | BV(10) | BV(9) | BV(8) | BV(7) | BV(6), GPIO_MODE_IN_FLOATING,                 
			    GPIO_SPEED_50MHZ);
			    
		//Port D
			stm32_gpioPinConfig(((struct stm32_gpio *)GPIOD_BASE),		
			    BV(2), GPIO_MODE_IN_FLOATING,                 
			    GPIO_SPEED_50MHZ);
			    
		
		/* Creation of a new process*/
		proc_new(reception_process, NULL, sizeof(stack_reception), stack_reception);
		
      	//Init readValues
		for(i=0;i<NUM_SENSORS;i++)  {
		  readValues[i]=0;
		}
      return;
}

void NORETURN reception_process(void)
{
  Timer timer_reception_process;
  int i;
  
  timer_setDelay(&timer_reception_process, us_to_ticks((utime_t)(reception_refresh)));
  timer_setEvent(&timer_reception_process);
  while(1) {
  	timer_add(&timer_reception_process);// Start process timer
	readSensors();
	for(i=0;i<NUM_SENSORS;i++)  {
	  if(readValues[0]==0) LED2_ON();
	  else LED2_OFF();
  	}
  	timer_waitEvent(&timer_reception_process); // Wait until the end of counting
  }
}

void readSensors(void)  {
  //Read all sensors

  readValues[0]=stm32_gpioPinRead(((struct stm32_gpio *)GPIOB_BASE), BV(9)); //read PB9
  readValues[1]=stm32_gpioPinRead(((struct stm32_gpio *)GPIOB_BASE), BV(8)); //read PB8
  readValues[2]=stm32_gpioPinRead(((struct stm32_gpio *)GPIOB_BASE), BV(7)); //read PB7
  readValues[3]=stm32_gpioPinRead(((struct stm32_gpio *)GPIOB_BASE), BV(6)); //read PB6
  readValues[4]=stm32_gpioPinRead(((struct stm32_gpio *)GPIOD_BASE), BV(2)); //read PD2
  readValues[5]=stm32_gpioPinRead(((struct stm32_gpio *)GPIOC_BASE), BV(12)); //read PC12
  readValues[6]=stm32_gpioPinRead(((struct stm32_gpio *)GPIOC_BASE), BV(11)); //read PC11
  readValues[7]=stm32_gpioPinRead(((struct stm32_gpio *)GPIOC_BASE), BV(10)); //read PC10
  readValues[8]=stm32_gpioPinRead(((struct stm32_gpio *)GPIOC_BASE), BV(9)); //read PC9
  readValues[9]=stm32_gpioPinRead(((struct stm32_gpio *)GPIOC_BASE), BV(8)); //read PC8
  readValues[10]=stm32_gpioPinRead(((struct stm32_gpio *)GPIOC_BASE), BV(7)); //read PC7
  readValues[11]=stm32_gpioPinRead(((struct stm32_gpio *)GPIOC_BASE), BV(6)); //read PC6
  readValues[12]=stm32_gpioPinRead(((struct stm32_gpio *)GPIOB_BASE), BV(15)); //read PB15
  readValues[13]=stm32_gpioPinRead(((struct stm32_gpio *)GPIOB_BASE), BV(14)); //read PB14
  readValues[14]=stm32_gpioPinRead(((struct stm32_gpio *)GPIOB_BASE), BV(13)); //read PB13
  readValues[15]=stm32_gpioPinRead(((struct stm32_gpio *)GPIOB_BASE), BV(12)); //read PB12

  return;
}
