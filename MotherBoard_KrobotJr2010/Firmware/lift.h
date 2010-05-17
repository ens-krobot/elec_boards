/*
 * Lift management on Krobot Jr
 * Xavier Lagorce
 */

#ifndef HEADER__LIFT
#define HEADER__LIFT

#include "ch.h"
#include "hal.h"
#include "stm32f10x.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

#define LIFT_UP    3273
#define LIFT_DOWN     0

void liftInit(void);
void liftGoto(uint16_t position);

#endif
