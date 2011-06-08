#include "intelligence.h"
#include "asservissement.h"
#include "encoder.h"
#include "motor.h"
#include "odometry.h"

#include <cfg/macros.h>
#include <drv/gpio_stm32.h>
#include "stm32lib/stm32f10x_tim.h"
#include "stm32lib/stm32f10x_rcc.h"
#include "stm32lib/stm32f10x.h"
#include <math.h>

PROC_DEFINE_STACK(stack_intelligence, KERN_MINSTACKSIZE * 4);

int etape=0;

void intelligenceInit(void)  {
	int i;
	
	holonome.xmax=0;
	holonome.vXmax=0.25;
	holonome.ax=0.25;
	holonome.ymax=-0;
	holonome.vYmax=0.3;
	holonome.ay=0.15;
	holonome.Tmax=20*M_PI;
	holonome.vTmax=3;
	holonome.aT=0.5;
	holonome.asserPosition=0;
	holonome.X=0;
	holonome.Y=0;
	holonome.T=0;
	holonome.Xstart=0;
	holonome.Ystart=0;
	holonome.Tstart=0;
	holonome.acceleration=0;
	holonome.vitesse=0;
	holonome.busy =0;
	
	for(i=0;i<NUM_MOTORS-1;i++)
		{
			holonome.Tprevious[i]=0;
		}
		
	/* Create a new child process */
        
        proc_new(intelligence_process, NULL, sizeof(stack_intelligence), stack_intelligence);
        
        
	return;
}

void NORETURN intelligence_process(void)
{
  Timer timer_intelligence_process;
  timer_setDelay(&timer_intelligence_process, us_to_ticks((utime_t)(intelligence_refresh)));
  timer_setEvent(&timer_intelligence_process);
  
  
  
  while(1) {
  	
  	timer_add(&timer_intelligence_process);// Start process timer
  	
  	makePath();
    timer_waitEvent(&timer_intelligence_process); // Wait until the end of counting
  }
  
   
}

void makePath(void)  {
	int i;
	if(holonome.busy==0)  { //On peut donc envoyer une nouvelle position
		
		holonome.X = holonome_odometry.X;
		holonome.Y = holonome_odometry.Y;
		
		//holonome.X = holonome.xmax;
		//holonome.Y = holonome.ymax;
		/*for(i=0;i<3;i++) {
			holonome.Tprevious[i]=0;
		}*/
		//holonome.asserPosition=0;
		switch(etape) {
			case 0:
				holonome.busy=1;
				holonome.asserPosition=0;
				holonome.xmax = -0.5;
				holonome.ymax = 0;
				holonome.vitesse = 0.5;
				holonome.acceleration=0.5;
				etape+=1;
				break;
			case 1:
				holonome.busy=1;
				holonome.asserPosition=0;
				holonome.xmax = -0.5;
				holonome.ymax = 0;
				holonome.vitesse = 0.5;
				holonome.acceleration=0.5;
				etape+=1;
				break;
			case 2:
				holonome.busy=1;
				holonome.asserPosition=0;
				holonome.xmax = -0.5;
				holonome.ymax = -0.5;
				holonome.vitesse = 0.5;
				holonome.acceleration=0.5;
				etape+=1;
				break;
			case 3:
				holonome.busy=1;
				holonome.asserPosition=0;
				holonome.xmax = -0.5;
				holonome.ymax = -0.5;
				holonome.vitesse = 0.5;
				holonome.acceleration=0.5;
				etape+=1;
				break;
			case 4:
				holonome.busy=1;
				holonome.asserPosition=0;
				holonome.xmax = 0;
				holonome.ymax = -0.5;
				holonome.vitesse = 0.5;
				holonome.acceleration=0.5;
				etape+=1;
				break;
			case 5:
				holonome.busy=1;
				holonome.asserPosition=0;
				holonome.xmax = 0;
				holonome.ymax = -0.5;
				holonome.vitesse = 0.5;
				holonome.acceleration=0.5;
				etape+=1;
				break;
			case 6:
				holonome.busy=1;
				holonome.asserPosition=0;
				holonome.xmax = 0;
				holonome.ymax = 0;
				holonome.vitesse = 0.5;
				holonome.acceleration=0.5;
				etape+=1;
				break;
			case 7:
				holonome.busy=1;
				holonome.asserPosition=0;
				holonome.xmax = 0;
				holonome.ymax = 0;
				holonome.vitesse = 0.5;
				holonome.acceleration=0.5;	
				etape=0;
				break;
		
			 
		}
		
		
	}
	
	
	return;
}
