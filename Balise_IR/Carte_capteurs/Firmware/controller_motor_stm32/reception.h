#ifndef HEADER__RECEPTION
#define HEADER__RECEPTION

#include <drv/gpio_stm32.h>
#include <drv/clock_stm32.h>
#include "hw/hw_led.h"
#include <drv/timer.h>

#include <stdlib.h>

#define PWM_56k 1286
#define NUM_SENSORS 16
#define NB_DETECT_MIN 5
#define reception_refresh 10000

uint16_t  readValues[NUM_SENSORS][2];
uint16_t  detectionState[NUM_SENSORS];


void receptionInit(void);

void enableClock(void);
void disableClock(void);


void readSensors(int); 
uint16_t refreshDetection(void);
uint16_t calculate_weight(uint16_t);

void NORETURN reception_process(void);


#endif
