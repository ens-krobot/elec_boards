#ifndef HEADER__EMISSION
#define HEADER__EMISSION

#include <drv/gpio_stm32.h>
#include <drv/clock_stm32.h>
#include "hw/hw_led.h"
#include <drv/timer.h>

#include <stdlib.h>

#define PWM_56k 1278
#define NUM_LEDS 16

#define CYCLE_TIME_US 17
#define LENGTH_WORD 10
#define EMISSION_TIME_HIGH 35*CYCLE_TIME_US
#define EMISSION_TIME_LOW 15*CYCLE_TIME_US

#define CYCLE_WAIT 100


void emissionInit(void);

void enableClock(void);
void disableClock(void);

void initLatch(void);

void writeSDI(unsigned short value); 
void writeCLK(unsigned short value); 
void writeLatch(unsigned short value); 
void writeCommande(unsigned short value); 

void NORETURN emission_process(void);

void initLatch(void);

void wait_clock(int nb_clk);

void selectLed(int num_led);
void nextLed(void);

void selectDoubleLed(int num_first_led);

#endif
