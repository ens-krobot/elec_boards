/*
 * Speed control
 * Xavier Lagorce
 */

#ifndef HEADER__SPEEDCONTROL
#define HEADER__SPEEDCONTROL

#define K_P   20
#define K_I    1
#define Te    50

#define K_v   (1000/Te)
#define K_Pn        K_P
#define K_In   5//(K_I)*Te

#define DEAD_ZONE       100
#define MAX_COMMAND   35000

#include "ch.h"
#include "motor.h"
#include "encoder.h"

extern volatile int ref_speeds[3];

void speedControlInit(void);
void sc_setRefSpeed(uint8_t motor, int speed);
int sc_getRealSpeed(uint8_t motor);
uint16_t sc_getPosition(uint8_t motor);

#endif
