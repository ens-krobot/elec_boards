#ifndef HEADER__avoid
#define HEADER__avoid

#include <drv/gpio_stm32.h>
#include <drv/clock_stm32.h>
#include "hw/hw_led.h"
#include <drv/timer.h>
#include "can_monitor.h"
#include "intelligence.h"
#include "asservissement.h"
#include "odometry.h"

#include <stdlib.h>

//Paramètre de l'asservissement

	#define avoid_refresh 40000
	#define NUM_SENSORS 8
	
	#define TRIGGER_150 1400
	#define TRIGGER_80 900
	#define TRIGGER_20 1000
	
	#define TRIGGER_1 2000
	#define TRIGGER_2 2000
	
	#define GP_150 3
	#define GP_80 2
	#define GP_20 1
	
	#define ADC_RESOLUTION 4096
	
	#define DETECTION_AREA M_PI/2

typedef struct {
	uint16_t value_int[NUM_SENSORS]; // sharp_sensors.value_int Sensors connected on the sensor card
	unsigned short state_trigger[NUM_SENSORS];
	unsigned short type_sensor[NUM_SENSORS];
	float circle_position[NUM_SENSORS];
} sharp;

sharp sharp_sensors;
unsigned short obstacle;

void avoidInit(void);
void refreshSensors(void);
void convertSensorsInMeters(void);

void NORETURN avoid_process(void);
void make_choice(void);
float reset_M_PI(float angle);

#endif
