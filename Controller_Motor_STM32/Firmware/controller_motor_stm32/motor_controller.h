/*
 * Motor controller
 * Xavier Lagorce
 */

#ifndef HEADER__MOTORCONTROLLER
#define HEADER__MOTORCONTROLLER

#include "motor.h"
#include "encoder.h"
#include "command_generator.h"

#define CONTROLLER_OK 0
#define CONTROLLER_ALREADY_USED 1

typedef struct {
  uint8_t motor;                    // Motor ID to control
  uint8_t encoder;                  // Encoder ID to measure motor position from
  float encoder_gain;               // Gain to convert encoder value unit to reference unit
  float G0;                         // DC motor static gain
  float tau;                        // DC motor time constant
  float k[2];                       // State control gain
  float l;                          // Reference factoring gain
  float l0[2];                      // State observer gain
  float T;                          // Sampling period of the controller in seconds
} motor_controller_params_t;

void motorControllerInit(void);
float mc_getSpeed(uint8_t motor);
float mc_getPosition(uint8_t motor);
void mc_setReference(uint8_t motor, command_generator_t *generator);


uint8_t mc_new_controller(motor_controller_params_t *cntr_params, command_generator_t *generator);
void mc_delete_controller(uint8_t motor);
#endif
