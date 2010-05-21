/*
 * Speed control
 * Xavier Lagorce
 */

#ifndef HEADER__SPEEDCONTROL
#define HEADER__SPEEDCONTROL

#define K_P    150
#define K_I      3
#define Tcomp   40 // speed will be computed on a Tcomp timesample
#define Te      50

#define K_v   (1000/Tcomp)

#define INTEG_MAX 20000

#include "ch.h"
#include "motor.h"
#include "encoder.h"

void speedControlInit(void);
void sc_setRefSpeed(uint8_t motor, int32_t speed);
int32_t sc_getRealSpeed(uint8_t motor);

#endif
