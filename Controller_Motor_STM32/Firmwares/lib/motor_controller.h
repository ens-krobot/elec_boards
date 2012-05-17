/*
 * Motor controller
 * Xavier Lagorce
 */

#ifndef HEADER__MOTORCONTROLLER
#define HEADER__MOTORCONTROLLER

#include "motor.h"
#include "encoder.h"
#include "command_generator.h"
#include <kern/proc.h>

#define CONTROLLER_OK 0
#define CONTROLLER_ALREADY_USED 1

#define CONTROLLER_MODE_NORMAL 1
#define CONTROLLER_MODE_HIL 2

/*
 * The motor_controller_params_t structure represents all the parameters needed
 * by the motor controller. They have to be set according to the motor being controlled
 */
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

/*
 * Initializes the motor controller subsystem. This won't start any controller but has to
 * be called prior to anything else.
 */
void motorControllerInit(void);

/*
 * Returns 'motor' 's last estimated speed.
 */
float mc_getSpeed(uint8_t motor);

/*
 * Returns 'motor' 's last position obtained from encoders.
 */
float mc_getPosition(uint8_t motor);

/*
 * Change the generator providing its position reference to 'motor' 's controller.
 */
void mc_setReference(uint8_t motor, command_generator_t *generator);

/*
 * Creates a new motor controller.
 *  - cntr_params : pointer to a structure containing controller parameters
 *  - generator : pointer to the generator providing position reference to the controller
 *  - mode : controller mode :
 *           x CONTROLLER_MODE_NORMAL : Normal, state space controller with state estimation
 *           x CONTROLLER_MODE_HIL : Hardware in the loop controller, to use in conjonction with a simulator
 */
uint8_t mc_new_controller(motor_controller_params_t *cntr_params, command_generator_t *generator, uint8_t mode);

/*
 * Asks for the deletion of an existing controller. This will stop the H-bridge
 * command output, so the control will stop immediately (but the controlling
 * thread will stop at its next sampling period).
 */
void mc_delete_controller(uint8_t motor);

/*
 * Changes the mode of an existing controller. This will create a new controller with the same parameters
 * as the old one but with the specifyed new mode.
 */
void mc_change_mode(uint8_t motor, uint8_t new_mode);

/*
 * Returns 1 if 'motor' 's controller is enabled.
 */
uint8_t mc_is_controller_enabled(uint8_t motor);
/*
 * Returns 1 if 'motor' 's controller's process is currently running
 */
uint8_t mc_is_controller_running(uint8_t motor);
/*
 * Returns 'motor' 's controller mode
 */
uint8_t mc_controller_mode(uint8_t motor);
/*
 * Suspends 'motor' 's controller
 */
void mc_suspend_controller(uint8_t motor);
/*
 * Get back on-line a controller
 */
void mc_reactivate_controller(uint8_t motor);

#endif
