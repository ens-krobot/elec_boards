/*
 * trajectory_controller.c
 * -----------------------
 * Copyright : (c) 2011, Xavier Lagorce <Xavier.Lagorce@crans.org>
 * Licence   : BSD3
 *
 * This file is a part of [kro]bot.
 */

#include "trajectory_controller.h"

#define TRAPEZOID_STATE_STOP     0
#define TRAPEZOID_STATE_ACC      1
#define TRAPEZOID_STATE_CONST    2
#define TRAPEZOID_STATE_DEC      3

#define SIGN(val) ((val) >= 0 ? 1 : -1)
#define SELECT_THRESHOLD(dir) ((dir) == 1 ? GEN_CALLBACK_SUP : GEN_CALLBACK_INF)
#define SELECT_THRESHOLD_DEC(dir) ((dir) == 1 ? GEN_CALLBACK_INF : GEN_CALLBACK_SUP)

typedef struct {
  float angle, speed, acceleration, init_val;
  int8_t dir;
  uint8_t state, is_triangle;
} tc_automaton_t;

typedef struct {
  uint8_t enabled;                  // Is this controller enabled
  uint8_t working;                  // Is this controller doing something ?
  command_generator_t position;     // Position generator (RAMP2)
  command_generator_t speed;        // Speed generator (RAMP)
  tc_automaton_t aut;               // Automaton to control movement
} trajectory_controller_t;


void trapezoid_callback(command_generator_t *generator);
void acc_stop_callback(command_generator_t *generator);

trajectory_controller_t controllers[4];

static inline uint8_t get_motor_index(uint8_t motor_id) {
  uint8_t motor_ind;

  for (motor_ind = 0; (motor_id >> (motor_ind+1)) != 0; motor_ind++) ;

  return motor_ind;
}

void trapezoid_callback(command_generator_t *generator) {
  trajectory_controller_t *cont = NULL;
  uint8_t i;

  // Select the correct controller depending on chich callbacl triggered
  for (i=0; i < 4; i++) {
    if (generator == &(controllers[i].position)
        || generator == &(controllers[i].speed) ) {
      cont = &controllers[i];
      break;
    }
  }
  // Exit if we have not found any...
  if (cont == NULL) {
    // Disable the callback to prevent repeated useless triggering
    remove_callback(generator);
    return;
  }

  // Unregistering callbacks of the current phase
  remove_callback(&cont->position);
  remove_callback(&cont->speed);

  // Automaton evolution
  switch (cont->aut.state) {
  case TRAPEZOID_STATE_STOP:
    // The automaton is already stopped, this shouldn't happen
    break;
  case TRAPEZOID_STATE_ACC:
    // Speed is inceasing, we have to go to the constant speed phase of the movement
    adjust_speed(&cont->speed, 0.);
    adjust_value(&cont->speed, cont->aut.speed);
    add_callback(&cont->position, SELECT_THRESHOLD(cont->aut.dir), cont->aut.angle - cont->aut.speed*cont->aut.speed/cont->aut.acceleration/2., trapezoid_callback);
    cont->aut.state = TRAPEZOID_STATE_CONST;
    break;
  case TRAPEZOID_STATE_CONST:
    // End of the constant speed phase, we have to slow down
    adjust_speed(&cont->speed, -cont->aut.acceleration);
    adjust_value(&cont->speed, cont->aut.speed);
    add_callback(&cont->position, SELECT_THRESHOLD(cont->aut.dir), cont->aut.angle, trapezoid_callback);
    add_callback(&cont->speed, SELECT_THRESHOLD_DEC(cont->aut.dir), 0.01*cont->aut.speed, trapezoid_callback);
    cont->aut.state = TRAPEZOID_STATE_DEC;
    break;
  case TRAPEZOID_STATE_DEC:
    // End of the movement, stop the motor
    adjust_speed(&cont->speed, 0.);
    adjust_value(&cont->speed, 0.);
    cont->aut.state = TRAPEZOID_STATE_STOP;
    cont->working = 0;
    break;
  }
}

void acc_stop_callback(command_generator_t *generator) {
  trajectory_controller_t *cont = NULL;
  uint8_t i;

  // Select the correct controller depending on chich callbacl triggered
  for (i=0; i < 4; i++) {
    if (generator == &(controllers[i].position)
        || generator == &(controllers[i].speed) ) {
      cont = &controllers[i];
      break;
    }
  }
  // Exit if we have not found any...
  if (cont == NULL) {
    // Disable the callback to prevent repeated useless triggering
    remove_callback(generator);
    return;
  }

  // Unregistering associated callbacks
  remove_callback(&cont->position);
  remove_callback(&cont->speed);

  // Stopping acceleration
  adjust_speed(&cont->speed, 0.);
  cont->working = 0;
}

void tc_init(void) {
  uint8_t i;

  // All Trajectory controllers are disabled
  for (i=0; i < 4; i++) {
    controllers[i].enabled = 0;
  }

  // Initialize motor controller system
  motorControllerInit();
}

void tc_new_controller(uint8_t motor, trajectory_controller_params_t *params) {
  trajectory_controller_t *cont;

  // Get corresponding controller
  cont = &controllers[get_motor_index(motor)];

  // Do nothing if the controller already exists
  if (cont->enabled == 1)
    return;

  // Create generators for position and speed manipulation
  new_ramp_generator(&cont->speed, 0., 0.);
  new_ramp2_generator(&cont->position, 0., &cont->speed);

  // Start control of drive motor
  mc_new_controller(motor,
                    params->encoder,
                    params->encoder_gain,
                    params->G0,
                    params->tau,
                    params->T,
                    params->k,
                    -params->k[0],
                    params->l0,
                    &cont->position);

  // Start the generator
  start_generator(&cont->position);
  start_generator(&cont->speed);

  // Initial state
  cont->aut.state = TRAPEZOID_STATE_STOP;
  cont->working = 0;
  cont->enabled = 1;
}

void tc_delete_controller(uint8_t motor) {
  trajectory_controller_t *cont;

  cont = &controllers[get_motor_index(motor)];

  if (cont->enabled == 1) {
    mc_delete_controller(motor);
    cont->working = 0;
    cont->enabled = 0;
  }
}

uint8_t tc_is_working(uint8_t motors) {
  uint8_t state = 0, i;

  // For each motor in motor, test if the corresponding controller is working
  for (i=0; i < 4; i++) {
    if ( ((motors & (1<<i)) != 0) && controllers[i].working == 1)
      state |= 1 << i;
  }

  return state;
}

void tc_goto(uint8_t motor, float angle, float speed, float acceleration) {
  float acc_dist, t_acc, t_end;
  trajectory_controller_t *cont;

  // Verify parameters
  if (angle == 0 || speed <= 0 || acceleration <= 0)
    return;

  // Get the controller and verifies it is enabled
  cont = &controllers[get_motor_index(motor)];
  if (!cont->enabled)
    return;

  // Compute some common parameters
  cont->aut.init_val = get_output_value(&cont->position);
  cont->aut.dir = SIGN(angle);
  cont->aut.angle = cont->aut.init_val + angle;
  cont->aut.acceleration = cont->aut.dir * acceleration;

  // Is the trapezoidal speed profile posible ?
  t_acc = speed / acceleration;
  t_end = (speed * speed + fabsf(angle) * acceleration) / (speed * acceleration);

  if (t_end > (2. * t_acc)) {
    // A trapezoidal speed profile is possible
    cont->aut.is_triangle = 0;

    // Compute trapezoid parameters
    cont->aut.speed = cont->aut.dir * speed;
    cont->aut.state = TRAPEZOID_STATE_ACC;

    // This is the distance during which the robot will accelerate
    acc_dist = cont->aut.dir * speed * speed / acceleration;

    // Set accelerations for the trapezoid's first phase and associated callbacks
    adjust_speed(&cont->speed, cont->aut.acceleration);
    add_callback(&cont->position, SELECT_THRESHOLD(cont->aut.dir), cont->aut.init_val + acc_dist, trapezoid_callback);
    add_callback(&cont->speed, SELECT_THRESHOLD(cont->aut.dir), cont->aut.speed, trapezoid_callback);
  } else {
    // A trapezoidal speed profile is not possible with the given acceleration, let's go for a triangle
    cont->aut.is_triangle = 1;

    // Compute triangle parameters
    cont->aut.speed = cont->aut.dir * sqrt(angle * cont->aut.acceleration);
    cont->aut.state = TRAPEZOID_STATE_CONST;

    // Set accelerations for the triangle's first phase and associated callbacks
    adjust_speed(&cont->speed, cont->aut.acceleration);
    add_callback(&cont->position, SELECT_THRESHOLD(cont->aut.dir), cont->aut.init_val + angle / 2.0, trapezoid_callback);
    add_callback(&cont->speed, SELECT_THRESHOLD(cont->aut.dir), cont->aut.speed, trapezoid_callback);
  }

  cont->working = 1;
}

void tc_move(tc_robot_t *robot, float distance, float speed, float acceleration) {
  float dis_s, spe_s, acc_s;

  // Let's pause the wheel's speed generators to synchronize movement start
  pause_generator(&controllers[get_motor_index(robot->left_wheel)].speed);
  pause_generator(&controllers[get_motor_index(robot->right_wheel)].speed);

  // Compute parameters
  dis_s = distance / robot->wheel_radius * 180.0 / M_PI;
  spe_s = speed / robot->wheel_radius * 180.0 / M_PI;
  acc_s = acceleration / robot->wheel_radius * 180.0 / M_PI;

  // Planify movements
  tc_goto(robot->left_wheel, dis_s, spe_s, acc_s);
  tc_goto(robot->right_wheel, dis_s, spe_s, acc_s);

  // Go
  start_generator(&controllers[get_motor_index(robot->left_wheel)].speed);
  start_generator(&controllers[get_motor_index(robot->right_wheel)].speed);
}

void tc_turn(tc_robot_t *robot, float angle, float speed, float acceleration) {
  float angle_s, spe_s, acc_s;

  // Let's pause the wheel's speed generators to synchronize movement start
  pause_generator(&controllers[get_motor_index(robot->left_wheel)].speed);
  pause_generator(&controllers[get_motor_index(robot->right_wheel)].speed);

  // Compute parameters
  angle_s = angle * robot->shaft_width / 2.0 / robot->wheel_radius;
  spe_s = speed * robot->shaft_width / 2.0 / robot->wheel_radius;
  acc_s = acceleration * robot->shaft_width / 2.0 / robot->wheel_radius;

  // Planify movements
  tc_goto(robot->left_wheel, -angle_s, spe_s, acc_s);
  tc_goto(robot->right_wheel, angle_s, spe_s, acc_s);

  // Go
  start_generator(&controllers[get_motor_index(robot->left_wheel)].speed);
  start_generator(&controllers[get_motor_index(robot->right_wheel)].speed);
}
