#ifndef HEADER__INTELLIGENCE
#define HEADER__INTELLIGENCE

#include <drv/gpio_stm32.h>
#include <drv/clock_stm32.h>
#include "hw/hw_led.h"
#include <drv/timer.h>

#include <stdlib.h>

//Paramètre de l'asservissement

	#define intelligence_refresh 50000
	
void makePath(void);
void intelligenceInit(void);

void NORETURN intelligence_process(void);

#endif
