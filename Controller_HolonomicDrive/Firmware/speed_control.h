/*
 * Speed control
 * Xavier Lagorce
 */

#ifndef HEADER__SPEEDCONTROL
#define HEADER__SPEEDCONTROL

#define K_P    170
#define K_I      4
#define Tcomp    5 // speed will be computed on a Tcomp timesample
#define Te      10

#define K_v   (1000/Tcomp)

#define INTEG_MAX 200000

#include "ch.h"
#include "motor.h"
#include "encoder.h"

void speedControlInit(void);
void sc_setRefSpeed(uint8_t motor, int32_t speed);
int32_t sc_getRealSpeed(uint8_t motor);

#endif
