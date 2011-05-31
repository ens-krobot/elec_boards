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

#define CONTROLLERS_STACK_SIZE (KERN_MINSTACKSIZE*16)

typedef struct
{
  uint8_t enable;                   // Is this controller enabled ?
  uint8_t running;                  // Is this controller running ?
  uint8_t mode;                     // Is this a real controller or an HIL controller
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
  float T;                          // sampling period in seconds
  cpu_stack_t *stack;               // controller process stack
  size_t stack_size;                // controller process stack size
} control_params_t;

static control_params_t controllers[4];
static cpu_stack_t controllers_stacks[4][(CONTROLLERS_STACK_SIZE + sizeof(cpu_stack_t) - 1) / sizeof(cpu_stack_t)];

static inline uint8_t get_motor_index(uint8_t motor_id) {
  uint8_t motor_ind;

  for (motor_ind = 0; (motor_id >> (motor_ind+1)) != 0; motor_ind++) ;

  return motor_ind;
}

static inline void motor_led_on(uint8_t motor_id) {
  switch (motor_id) {
  case MOTOR1:
    LED1_ON();
    break;
  case MOTOR2:
    LED2_ON();
    break;
  case MOTOR3:
    LED3_ON();
    break;
  case MOTOR4:
    LED4_ON();
    break;
  }
}

static inline void motor_led_off(uint8_t motor_id) {
  switch (motor_id) {
  case MOTOR1:
    LED1_OFF();
    break;
  case MOTOR2:
    LED2_OFF();
    break;
  case MOTOR3:
    LED3_OFF();
    break;
  case MOTOR4:
    LED4_OFF();
    break;
  }
}

// Process functions for control and simulation
static void NORETURN motorController_process(void);
static void NORETURN motorController_HIL_process(void);

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

  // Indicate we are running
  params->running = 1;

  // configure timer
  timer_setDelay(&timer, ms_to_ticks((mtime_t)(params->T*1000)));
  timer_setEvent(&timer);

  // Switch off motor LED
  motor_led_off(params->motor);

  while (1) {
    if (params->enable == 0) {
      params->running = 0;
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

/* Hardware in the loop controller process
 * This process is to be used for Hardware in the loop simulation. It will consider
 * that the motor control process is perfect and send position data to the simulator
 * through the CAN bus every sampling period.
 */
static void NORETURN motorController_HIL_process(void) {
  control_params_t *params;
  Timer timer;
  float last_t = 0;
  uint8_t led_state = 0;

  // get data
  params = (control_params_t *) proc_currentUserData();

  // Indicate we are running
  params->running = 1;

  // configure timer
  timer_setDelay(&timer, ms_to_ticks((time_t)(params->T*1000)));
  timer_setEvent(&timer);

  while (1) {
    if (params->enable == 0) {
      params->running = 0;
      proc_exit();
    } else {
      timer_add(&timer);

      if (last_t >= 0.2) {
        led_state = !led_state;
        if (led_state) {
          motor_led_on(params->motor);
        } else {
          motor_led_off(params->motor);
        }
        last_t -= 0.2;
      }

      // Compute "state estimation"
      params->last_estimate[0] = get_output_value(params->reference);
      params->last_estimate[1] = (params->last_estimate[0] - params->last_output) / params->T;

      // Keep some infos
      params->last_output = params->last_estimate[0];
    }
    timer_waitEvent(&timer); // Wait for the remaining of the sample period
    last_t += params->T;
  }
}

void motorControllerInit() {
  uint8_t motor_ind;

  for (motor_ind = 0; motor_ind < 4; motor_ind++) {
    controllers[motor_ind].enable = 0;
    controllers[motor_ind].running = 0;
    controllers[motor_ind].stack = &controllers_stacks[motor_ind][0];
    controllers[motor_ind].stack_size = sizeof(controllers_stacks[motor_ind]);
  }

  encodersInit();
  motorsInit();
}

uint8_t mc_is_controller_enabled(uint8_t motor) {
  control_params_t *params;

  params = &(controllers[get_motor_index(motor)]);
  return params->enable;
}

uint8_t mc_is_controller_running(uint8_t motor) {
  control_params_t *params;

  params = &(controllers[get_motor_index(motor)]);
  return params->running;
}

uint8_t mc_controller_mode(uint8_t motor) {
  control_params_t *params;

  params = &(controllers[get_motor_index(motor)]);
  if (params->enable)
    return params->mode;
  else
    return 0;
}

uint8_t mc_new_controller(motor_controller_params_t *cntr_params, command_generator_t *generator, uint8_t mode) {
  uint8_t motor_ind;
  control_params_t *params;
  float tau, T;

  // Find the motor index
  motor_ind = get_motor_index(cntr_params->motor);
  params = &(controllers[motor_ind]);

  if (params->enable == 0 && params->running == 0) {
    // define user parameters
    params->motor = cntr_params->motor;
    params->encoder = cntr_params->encoder;
    params->encoder_gain = cntr_params->encoder_gain;
    params->tau = cntr_params->tau;
    params->T = cntr_params->T;
    params->k[0] = cntr_params->k[0]; params->k[1] = cntr_params->k[1];
    params->l = cntr_params->l;
    params->l0[0] = cntr_params->l0[0]; params->l0[1] = cntr_params->l0[1];

    // compute other parameters
    params->last_command = 0;
    params->last_estimate[0] = get_output_value(generator); params->last_estimate[1] = 0;
    params->last_output = params->last_estimate[0];
    params->last_encoder_pos = getEncoderPosition(cntr_params->encoder);
    params->reference = generator;

    tau = cntr_params->tau;
    T = cntr_params->T;
    params->F[0] = 1;
    params->F[1] = tau*(1-exp(-T/tau));
    params->F[2] = 0;
    params->F[3] = exp(-T/tau);
    params->G[0] = cntr_params->G0*(T+tau*exp(-T/tau)-tau);
    params->G[1] = cntr_params->G0*(1-exp(-T/tau));

    // enable the controller
    params->enable = 1;

    // start the correct controller depending on the mode
    params->mode = mode;
    if (mode == CONTROLLER_MODE_NORMAL) {
      proc_new(motorController_process, params, params->stack_size, params->stack);
      enableMotor(cntr_params->motor);
      motorSetSpeed(cntr_params->motor, 0);
    }
    else {
      proc_new(motorController_HIL_process, params, params->stack_size, params->stack);
    }

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
    if (params->mode == CONTROLLER_MODE_NORMAL)
      disableMotor(motor);
  }
}

void mc_change_mode(uint8_t motor, uint8_t new_mode) {
  control_params_t *params;

  params = &(controllers[get_motor_index(motor)]);

  // Do something only if the controller is enabled and in a different mode
  if (params->enable && params->mode != new_mode) {
    // Delete the old controller
    mc_delete_controller(motor);
    // Wait for the controller process to stop
    while (params->running)
      cpu_relax();
    // Recreate the controller in the correct mode
    params->last_command = 0;
    params->last_estimate[0] = get_output_value(params->reference); params->last_estimate[1] = 0;
    params->last_output = params->last_estimate[0];
    params->last_encoder_pos = getEncoderPosition(params->encoder);
    params->mode = new_mode;
    // enable the controller
    params->enable = 1;

    // start the correct controller depending on the mode
    if (new_mode == CONTROLLER_MODE_NORMAL) {
      proc_new(motorController_process, params, params->stack_size, params->stack);
      enableMotor(params->motor);
      motorSetSpeed(params->motor, 0);
    }
    else {
      proc_new(motorController_HIL_process, params, params->stack_size, params->stack);
    }
  }
}
