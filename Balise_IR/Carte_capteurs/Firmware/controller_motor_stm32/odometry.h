#ifndef HEADER__ODOMETRY
#define HEADER__ODOMETRY

#include <drv/gpio_stm32.h>
#include <drv/clock_stm32.h>
#include "hw/hw_led.h"
#include <drv/timer.h>

#include <stdlib.h>

//Paramètre de l'asservissement

	#define odometry_refresh 2500
	
typedef struct {

	int64_t previousCounter[3];
	
	//Position du robot sur la table
	float X;
	float Y;
	float T;

	
} robot_odometry;
	
robot_odometry holonome_odometry;


void odometryInit(void);
void refreshOdometry(void);

void NORETURN odometry_process(void);

#endif
