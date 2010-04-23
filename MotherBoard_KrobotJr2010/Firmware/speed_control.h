/*
 * Speed control
 * Xavier Lagorce
 */

#ifndef HEADER__SPEEDCONTROL
#define HEADER__SPEEDCONTROL

#define K_P     15
#define K_I      1
#define Tcomp   30 // speed will be computed on a Tcomp timesample
#define Te      50

#define K_v   (1000/Tcomp)
#define K_Pn        K_P
#define K_In   10//(K_I)*Te

#define DEAD_ZONE       100
#define MAX_COMMAND   35500

#include "ch.h"
#include "motor.h"
#include "encoder.h"

void speedControlInit(void);
void sc_setRefSpeed(uint8_t motor, int32_t speed);
int32_t sc_getRealSpeed(uint8_t motor);

#endif
