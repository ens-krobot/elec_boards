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
		  readValues[i][0]=0;
		  readValues[i][1]=0;
		  detectionState[i]=0;
		}
      return;
}

void NORETURN reception_process(void)
{
  Timer timer_reception_process;
  int i;
  float theta;
  uint16_t control;
  
  uint16_t opponentSensor;
  uint16_t maxWeight;
  
 
  timer_setDelay(&timer_reception_process, us_to_ticks((utime_t)(reception_refresh)));
  timer_setEvent(&timer_reception_process);
  while(1) {

  	timer_add(&timer_reception_process);// Start process timer

	uint16_t weight_tab[NUM_SENSORS]={0};

	control=refreshDetection();

	if (control >= NB_DETECT_MIN){		// process only when enough sensors are enlightened

		maxWeight = 0;

		for(i=0 ; i<NUM_SENSORS ; i++){

			weight_tab[i] = calculate_weight(i);

			if (weight_tab[i] > maxWeight){

				maxWeight = weight_tab[i];
				opponentSensor = i;

			}
		}
	
	theta = 2*M_PI - (2*M_PI/32 + opponentSensor * 2*M_PI/16);

	}

	
	if (control >= NB_DETECT_MIN) LED2_ON();
	else LED2_OFF();
	if (theta < M_PI/2) LED1_ON();
	else LED1_OFF();
  	
  	timer_waitEvent(&timer_reception_process); // Wait until the end of counting
  }
}

void readSensors(int indice)  {
  //Read all sensors

  readValues[0][indice]=stm32_gpioPinRead(((struct stm32_gpio *)GPIOB_BASE), BV(9)); //read PB9
  readValues[1][indice]=stm32_gpioPinRead(((struct stm32_gpio *)GPIOB_BASE), BV(8)); //read PB8
  readValues[2][indice]=stm32_gpioPinRead(((struct stm32_gpio *)GPIOB_BASE), BV(7)); //read PB7
  readValues[3][indice]=stm32_gpioPinRead(((struct stm32_gpio *)GPIOB_BASE), BV(6)); //read PB6
  readValues[4][indice]=stm32_gpioPinRead(((struct stm32_gpio *)GPIOD_BASE), BV(2)); //read PD2
  readValues[5][indice]=stm32_gpioPinRead(((struct stm32_gpio *)GPIOC_BASE), BV(12)); //read PC12
  readValues[6][indice]=stm32_gpioPinRead(((struct stm32_gpio *)GPIOC_BASE), BV(11)); //read PC11
  readValues[7][indice]=stm32_gpioPinRead(((struct stm32_gpio *)GPIOC_BASE), BV(10)); //read PC10
  readValues[8][indice]=stm32_gpioPinRead(((struct stm32_gpio *)GPIOC_BASE), BV(9)); //read PC9
  readValues[9][indice]=stm32_gpioPinRead(((struct stm32_gpio *)GPIOC_BASE), BV(8)); //read PC8
  readValues[10][indice]=stm32_gpioPinRead(((struct stm32_gpio *)GPIOC_BASE), BV(7)); //read PC7
  readValues[11][indice]=stm32_gpioPinRead(((struct stm32_gpio *)GPIOC_BASE), BV(6)); //read PC6
  readValues[12][indice]=stm32_gpioPinRead(((struct stm32_gpio *)GPIOB_BASE), BV(15)); //read PB15
  readValues[13][indice]=stm32_gpioPinRead(((struct stm32_gpio *)GPIOB_BASE), BV(14)); //read PB14
  readValues[14][indice]=stm32_gpioPinRead(((struct stm32_gpio *)GPIOB_BASE), BV(13)); //read PB13
  readValues[15][indice]=stm32_gpioPinRead(((struct stm32_gpio *)GPIOB_BASE), BV(12)); //read PB12

  return;
}

uint16_t refreshDetection(void){

	int i;
	uint8_t control_sum=0;

	readSensors(0);
	timer_delay(1);
	readSensors(1);
	
	for(i=0 ; i < NUM_SENSORS ; i++){

		if (readValues[i][0]!=readValues[i][1])	detectionState[i] = 1;
		else detectionState[i] = 0;
		control_sum += detectionState[i];
	}

	

	return control_sum;
}

uint16_t calculate_weight(uint16_t k){

	int i;
	uint16_t weight=0;

	for(i=0 ; i<NUM_SENSORS ; i++){

		if((abs(k-i) < 5) & (i!=k)){

			weight += detectionState[i]*((uint16_t)pow(3,5-abs(k-i)));

		}
	
	}

	return weight;

}
