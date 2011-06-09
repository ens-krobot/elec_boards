#include "avoid.h"
#include "intelligence.h"
#include "asservissement.h"
#include "encoder.h"
#include "motor.h"
#include "avoid.h"

#include <cfg/macros.h>
#include <drv/gpio_stm32.h>
#include "stm32lib/stm32f10x_tim.h"
#include "stm32lib/stm32f10x_rcc.h"
#include "stm32lib/stm32f10x.h"
#include <math.h>

PROC_DEFINE_STACK(stack_avoid, KERN_MINSTACKSIZE * 4);
int actual_state;
float xmax, ymax, Tmax;
int num_wait; 
unsigned short stop_yet;

void avoidInit(void)  {
	
		
	/* Create a new child process */
        
        /*Initialize type of sharp_sensors sensors */
        sharp_sensors.type_sensor[0]=GP_80;
        sharp_sensors.type_sensor[1]=0;
        sharp_sensors.type_sensor[2]=GP_80;
        sharp_sensors.type_sensor[3]=GP_80;
        sharp_sensors.type_sensor[4]=GP_150;
        sharp_sensors.type_sensor[5]=GP_150;
        sharp_sensors.type_sensor[6]=GP_80;
        sharp_sensors.type_sensor[7]=GP_80;
        
        sharp_sensors.circle_position[0]=0;
        sharp_sensors.circle_position[1]=0;
        sharp_sensors.circle_position[2]=M_PI/3;
        sharp_sensors.circle_position[3]=2*M_PI/3;
        sharp_sensors.circle_position[4]=M_PI;
        sharp_sensors.circle_position[5]=M_PI;
        sharp_sensors.circle_position[6]=4*M_PI/3;
        sharp_sensors.circle_position[7]=-M_PI/3;
        
        proc_new(avoid_process, NULL, sizeof(stack_avoid), stack_avoid);
        
        num_wait = 0;
        
        
        
	return;
}

void NORETURN avoid_process(void)
{
  Timer timer_avoid_process;
  timer_setDelay(&timer_avoid_process, us_to_ticks((utime_t)(avoid_refresh)));
  timer_setEvent(&timer_avoid_process);
  
  
  
  while(1) {
  	
  	timer_add(&timer_avoid_process);// Start process timer
  	refreshSensors();
  	//if(sharp_sensors.state_trigger[7]==1) LED2_ON();
  	//else LED2_OFF();
  	make_choice();
    timer_waitEvent(&timer_avoid_process); // Wait until the end of counting
  }
  
   
}

void refreshSensors(void)  {

	int i;

	//Refresh value from ADC
		sharp_sensors.value_int[0] = adc1_msg.p.val1;
		sharp_sensors.value_int[1] = adc1_msg.p.val2;
		sharp_sensors.value_int[2] = adc1_msg.p.val3;
		sharp_sensors.value_int[3] = adc1_msg.p.val4;
		sharp_sensors.value_int[4] = adc2_msg.p.val1;
		sharp_sensors.value_int[5] = adc2_msg.p.val2;
		sharp_sensors.value_int[6] = adc2_msg.p.val3;
		sharp_sensors.value_int[7] = adc2_msg.p.val4;
	
	//Look for the trigger
	
		for(i=2; i<NUM_SENSORS;i++)  {
			switch(sharp_sensors.type_sensor[i])  {
				case GP_150:
					if(sharp_sensors.value_int[i]>TRIGGER_150) sharp_sensors.state_trigger[i]=1;
					else sharp_sensors.state_trigger[i]=0;
					break;
				case GP_80:
					if(sharp_sensors.value_int[i]>TRIGGER_80) sharp_sensors.state_trigger[i]=1;
					else sharp_sensors.state_trigger[i]=0;
					break;
				case GP_20:
					if(sharp_sensors.value_int[i]>TRIGGER_20) sharp_sensors.state_trigger[i]=1;
					else sharp_sensors.state_trigger[i]=0;
					break;
			}
			
			
					if(sharp_sensors.value_int[0]>TRIGGER_1) sharp_sensors.state_trigger[0]=1;
					else sharp_sensors.state_trigger[0]=0;
					
					if(sharp_sensors.value_int[1]>TRIGGER_2) sharp_sensors.state_trigger[1]=1;
					else sharp_sensors.state_trigger[1]=0;
					
		}
		
	
	return;
}	

void make_choice(void)  {
	int i;
	unsigned short result=0;
	float angle;
	
	for(i=0;i<NUM_SENSORS;i++)  {
		if(sharp_sensors.state_trigger[i]==1)  {
			//Trigger is ON	
			angle = reset_M_PI((sharp_sensors.circle_position[i]+holonome_odometry.T)-holonome.theta);
			if(abs(angle)<DETECTION_AREA/2 || abs((sharp_sensors.circle_position[i]-holonome_odometry.T)-holonome.theta-0*M_PI)<DETECTION_AREA/2)  {
				result=1;
				//LED2_ON();
				break;
			}
			//else LED2_OFF();
		}
	}
	if(result==1)  {
		if(stop_yet==0)  {
			holonome.asserPosition=0;
			xmax = holonome.xmax;
			ymax = holonome.ymax;
			Tmax = holonome.Tmax;
			holonome.X = holonome_odometry.X;
			holonome.Y = holonome_odometry.Y;
			holonome.T = holonome_odometry.T;
			holonome.xmax = holonome_odometry.X;
			holonome.ymax = holonome_odometry.Y;
			holonome.Tmax = holonome_odometry.T;
			if(num_wait>=0) stop_yet=1;
			num_wait++;
			obstacle=1;
		}
	}
	else {
		if(stop_yet==1)  {
			holonome.asserPosition = 0;
			holonome.X = holonome_odometry.X;
			holonome.Y = holonome_odometry.Y;
			holonome.T = holonome_odometry.T;
			holonome.xmax = xmax;
			holonome.ymax = ymax;
			holonome.Tmax = Tmax;
			stop_yet=0;
			num_wait=0;
			obstacle=0;
		}
	}
	result=0;
	return;
}

float reset_M_PI(float angle)  {
	if(angle > M_PI)  {
		while(angle > M_PI)  {
			angle -= 2*M_PI;
		}
	}
	else  {
		while(angle < -M_PI) {
			angle += 2*M_PI;
		}
	}
	
	return angle;
}	
