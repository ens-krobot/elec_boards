/*
 * Motor controller
 * Xavier Lagorce
 */

#include "motor_controller.h"
#include <kern/proc.h>
#include <drv/timer.h>
#include <math.h>

#ifndef MAX
 #define MAX(x,y) ((x) > (y) ? (x) : (y))
#endif
#ifndef MIN
 #define MIN(x,y) ((x) > (y) ? (y) : (x))
#endif

typedef struct
{
  uint8_t enable;                   // Is this controller enabled ?
  uint8_t motor;                    // Motor ID to control
  uint8_t encoder;                  // Encoder ID to measure motor position from
  float encoder_gain;               // Gain to convert encoder value unit to reference unit
  command_generator_t *reference;   // Goal position
  float tau;                        // DC motor time constant
  float k[2];                       // State control gain
  float l;                          // Reference factoring gain
  float l0[2];                      // State observer gain
  float last_command;               // Last applyed command
  float last_estimate[2];           // Last state estimate
  float last_output;                // Last measured output
  uint16_t last_encoder_pos;        // Last encoder position measured
  float F[4];                       // evolution matrix
  float G[2];                       // command application matrix
  ticks_t T;                        // sampling period in systicks
} control_params_t;

control_params_t controllers[4];

static inline uint8_t get_motor_index(uint8_t motor_id) {
  uint8_t motor_ind;

  for (motor_ind = 0; (motor_id >> (motor_ind+1)) != 0; motor_ind++) ;

  return motor_ind;
}

// Callback for control
static void NORETURN motorController_process(void);

float mc_getSpeed(uint8_t motor) {

  switch(motor) {
    case MOTOR1:
      return controllers[0].last_estimate[1];
      break;
    case MOTOR2:
      return controllers[1].last_estimate[1];
      break;
    case MOTOR3:
      return controllers[2].last_estimate[1];
      break;
    case MOTOR4:
      return controllers[3].last_estimate[1];
      break;
    default:
      return 0;
  }
}

float mc_getPosition(uint8_t motor) {

  switch(motor) {
    case MOTOR1:
      return controllers[0].last_output;
      break;
    case MOTOR2:
      return controllers[1].last_output;
      break;
    case MOTOR3:
      return controllers[2].last_output;
      break;
    case MOTOR4:
      return controllers[3].last_output;
      break;
    default:
      return 0;
  }
}

void mc_setReference(uint8_t motor, command_generator_t *generator) {

  switch(motor) {
    case MOTOR1:
      controllers[0].reference = generator;
      break;
    case MOTOR2:
      controllers[1].reference = generator;
      break;
    case MOTOR3:
      controllers[2].reference = generator;
      break;
    case MOTOR4:
      controllers[3].reference = generator;
      break;
  }
}

// Controller process
// this process will control one DC motor in state space with state estimation
static void NORETURN motorController_process(void) {
  control_params_t *params;
  float estimate[2], encoder_pos, delta;
  Timer timer;

  // get data
  params = (control_params_t *) proc_currentUserData();

  // configure timer
  timer_setDelay(&timer, ms_to_ticks(params->T));
  timer_setEvent(&timer);

  while (1) {
    if (params->enable == 0) {
      proc_exit();
    } else {
      timer_add(&timer);

      // Measure motor rotation
      encoder_pos = (float)getEncoderPosition(params->encoder);
      delta = encoder_pos - params->last_encoder_pos;
      if (getEncoderDirection(params->encoder)  == ENCODER_DIR_UP) {
        if (delta < 0)
          delta = delta + 65535;
      } else {
        if (delta > 0)
          delta = delta - 65535;
      }

      // New state vector estimation
      estimate[0] = (params->F[0]-params->l0[0])*params->last_estimate[0] + params->F[1]*params->last_estimate[1]
        + params->G[0]*params->last_command + params->l0[0]*params->last_output;
      estimate[1] = (params->F[2]-params->l0[1])*params->last_estimate[0] + params->F[3]*params->last_estimate[1]
        + params->G[1]*params->last_command + params->l0[1]*params->last_output;

      // Compute motor rotation
      params->last_output += params->encoder_gain*delta;

      // Command computation
      params->last_command = params->l*get_output_value(params->reference) + params->k[0]*estimate[0] + params->k[1]*estimate[1];

      // Keep estimate and data
      params->last_estimate[0] = estimate[0];
      params->last_estimate[1] = estimate[1];
      params->last_encoder_pos = encoder_pos;

      // Apply command
      motorSetSpeed(params->motor, (int32_t)params->last_command);
    }
    timer_waitEvent(&timer); // Wait for the remaining of the sample period
  }
}

void speedControlInit() {
  uint8_t motor_ind;

  for (motor_ind = 0; motor_ind < 4; motor_ind++)
    controllers[motor_ind].enable = 0;

  encodersInit();
  motorsInit();
}

uint8_t mc_new_controller(uint8_t motor, uint8_t encoder, float encoder_gain, float G0, float tau, float T, float *k, float l, float *l0, command_generator_t *generator) {
  uint8_t motor_ind;
  control_params_t *params;

  // Find the motor index
  motor_ind = get_motor_index(motor);
  params = &(controllers[motor_ind]);

  if (params->enable == 0) {
    // define user parameters
    params->motor = motor;
    params->encoder = encoder;
    params->encoder_gain = encoder_gain;
    params->tau = tau;
    params->T = ms_to_ticks((mtime_t)(T*1000));
    params->k[0] = k[0]; params->k[1] = k[1];
    params->l = l;
    params->l0[0] = l0[0]; params->l0[1] = l0[1];

    // compute other parameters
    params->last_command = 0;
    params->last_estimate[0] = 0; params->last_estimate[1] = 0;
    params->last_output = params->last_estimate[0];
    params->last_encoder_pos = getEncoderPosition(encoder);
    params->reference = generator;

    params->F[0] = 1;
    params->F[1] = tau*(1-exp(-T/tau));
    params->F[2] = 0;
    params->F[3] = exp(-T/tau);
    params->G[0] = G0*(T+tau*exp(-T/tau)-tau);
    params->G[1] = G0*(1-exp(-T/tau));

    // enable the controller
    params->enable = 1;
    enableMotor(motor);
    motorSetSpeed(motor, 0);

    // start the controller
    proc_new(motorController_process, params, KERN_MINSTACKSIZE * 16, NULL);

    return CONTROLLER_OK;
  } else {
    // Motor already controlled, return an error
    return CONTROLLER_ALREADY_USED;
  }

  // Should never be here
  return CONTROLLER_ALREADY_USED;
}

void mc_delete_controller(uint8_t motor) {
  uint8_t motor_ind;
  control_params_t *params;

  // Find the motor index
  motor_ind = get_motor_index(motor);
  params = &(controllers[motor_ind]);

  // Disable the controller and the power output
  if (params->enable != 0) {
    params->enable = 0;
    disableMotor(motor);
  }
}
