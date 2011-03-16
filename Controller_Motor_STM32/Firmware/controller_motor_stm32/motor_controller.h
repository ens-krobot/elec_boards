/*
 * Motor controller
 * Xavier Lagorce
 */

#ifndef HEADER__MOTORCONTROLLER
#define HEADER__MOTORCONTROLLER

#include "motor.h"
#include "encoder.h"

#define CONTROLLER_OK 0
#define CONTROLLER_ALREADY_USED 1

void speedControlInit(void);
float mc_getSpeed(uint8_t motor);
float mc_getPosition(uint8_t motor);
void mc_gotoPosition(uint8_t motor, float position);


uint8_t mc_new_controller(uint8_t motor, uint8_t encoder, float encoder_gain, float G0, float tau, float T, float *k, float l, float *l0);
void mc_delete_controller(uint8_t motor);
#endif
