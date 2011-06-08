#include "odometry.h"
#include "intelligence.h"
#include "asservissement.h"
#include "encoder.h"
#include "motor.h"

#include <cfg/macros.h>
#include <drv/gpio_stm32.h>
#include "stm32lib/stm32f10x_tim.h"
#include "stm32lib/stm32f10x_rcc.h"
#include "stm32lib/stm32f10x.h"
#include <math.h>

PROC_DEFINE_STACK(stack_odometry, KERN_MINSTACKSIZE * 4);


void odometryInit(void)  {
	int i;
	
	holonome_odometry.X=0;
	holonome_odometry.Y=0;
	holonome_odometry.T=0;
	
	for(i=0;i<NUM_MOTORS-1;i++)
		{
			holonome_odometry.previousCounter[i]=0;
		}
		
	/* Create a new child process */
        
        proc_new(odometry_process, NULL, sizeof(stack_odometry), stack_odometry);
        
        
	return;
}

void NORETURN odometry_process(void)
{
  Timer timer_odometry_process;
  timer_setDelay(&timer_odometry_process, us_to_ticks((utime_t)(odometry_refresh)));
  timer_setEvent(&timer_odometry_process);
  
  
  
  while(1) {
  	
  	timer_add(&timer_odometry_process);// Start process timer
  	refreshOdometry();
    timer_waitEvent(&timer_odometry_process); // Wait until the end of counting
  }
  
   
}
/*
void refreshOdometry()  {
	int i;
	int64_t dTheta[3];
	
	
	// Rafraichissement des encodeurs
	refreshEncoderStatus(ENCODER1 | ENCODER2 | ENCODER3 | ENCODER4);

	// Prise en compte de la variation d'impulsions
	for(i=0; i<NUM_MOTORS;i++)  {
		if(1<<i & MOTOR_CONTROL1) {
			dTheta[0] = encoders_status[i].globalCounter - holonome_odometry.previousCounter[0];
			holonome_odometry.previousCounter[0] = encoders_status[i].globalCounter;
		}
		
		if(1<<i & MOTOR_CONTROL2) {
			dTheta[1] = encoders_status[i].globalCounter - holonome_odometry.previousCounter[1];
			holonome_odometry.previousCounter[1] = encoders_status[i].globalCounter;
		}
		
		if(1<<i & MOTOR_CONTROL3) {
			dTheta[2] = encoders_status[i].globalCounter - holonome_odometry.previousCounter[2];
			holonome_odometry.previousCounter[2] = encoders_status[i].globalCounter;
		}
	}
	// T : position angulaire
	holonome_odometry.T += -R/(3*L)*(convertImpulsionsInRad( dTheta[0]+dTheta[1]+dTheta[2],0));
	if(holonome_odometry.T >=  0.01) LED2_ON(); 
	else LED2_OFF();
	// Y
	holonome_odometry.Y += R/3*(convertImpulsionsInRad( -2*dTheta[0]+dTheta[1]+dTheta[2],0));
	// X
	holonome_odometry.X += R/(sqrt(3))*(convertImpulsionsInRad( dTheta[1]-dTheta[2],0));
	return;

}*/

void refreshOdometry()  {
	int i;
	float dTheta[3];
	float X1, Y1;
	
	// Rafraichissement des encodeurs
	refreshEncoderStatus(ENCODER1 | ENCODER2 | ENCODER3 | ENCODER4);

	// Prise en compte de la variation d'impulsions
	for(i=0; i<NUM_MOTORS;i++)  {
		if(1<<i & MOTOR_CONTROL1) {
			dTheta[0] = convertImpulsionsInRad(encoders_status[i].globalCounter,0) - convertImpulsionsInRad(holonome_odometry.previousCounter[0],0);
			holonome_odometry.previousCounter[0] = encoders_status[i].globalCounter;
		}
		
		if(1<<i & MOTOR_CONTROL2) {
			dTheta[1] = convertImpulsionsInRad(encoders_status[i].globalCounter,0) - convertImpulsionsInRad(holonome_odometry.previousCounter[1],0);
			holonome_odometry.previousCounter[1] = encoders_status[i].globalCounter;
		}
		
		if(1<<i & MOTOR_CONTROL3) {
			dTheta[2] = convertImpulsionsInRad(encoders_status[i].globalCounter,0) - convertImpulsionsInRad(holonome_odometry.previousCounter[2],0);
			holonome_odometry.previousCounter[2] = encoders_status[i].globalCounter;
		}
	}
	// T : position angulaire
	holonome_odometry.T += -R/(3)*( dTheta[0]/L1+dTheta[1]/L2+dTheta[2]/L3);
	if(abs(holonome_odometry.T) >=  0.01) LED2_ON(); 
	else LED2_OFF();
	X1 = R/(sqrt(3))*(dTheta[1]-dTheta[2]);
	Y1 = R/3*( -2*dTheta[0]+dTheta[1]+dTheta[2]);
	
	// Y
	
	holonome_odometry.Y += sin(holonome_odometry.T)*X1+ cos(holonome_odometry.T)*Y1;
	// X
	holonome_odometry.X += cos(holonome_odometry.T)*X1- sin(holonome_odometry.T)*Y1;
	return;

}

