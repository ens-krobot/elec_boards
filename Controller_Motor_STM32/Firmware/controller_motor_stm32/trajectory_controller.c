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
#define TRAPEZOID_STATE_TRIANGLE 4

#define SIGN(val) ((val) >= 0 ? 1 : -1)
#define SELECT_THRESHOLD(dir) ((dir) == 1 ? GEN_CALLBACK_SUP : GEN_CALLBACK_INF)
#define SELECT_THRESHOLD_DEC(dir) ((dir) == 1 ? GEN_CALLBACK_INF : GEN_CALLBACK_SUP)

void trapezoid_callback(command_generator_t *generator);

typedef struct {
  float angle, speed, acceleration, init_val;
  int8_t dir;
  uint8_t state, is_triangle;
} ramp_automaton_t;

command_generator_t right_wheel, left_wheel, right_wheel_speed, left_wheel_speed;
ramp_automaton_t right_trap, left_trap;

uint8_t controller_state;

void trapezoid_callback(command_generator_t *generator) {
  ramp_automaton_t *automaton;
  command_generator_t *trap, *trap_speed;

  // Select the correct trapezoid depending on which callback triggered
  if (generator == &right_wheel || generator == &right_wheel_speed) {
    automaton = &right_trap;
    trap = &right_wheel;
    trap_speed = &right_wheel_speed;
  } else if (generator == &left_wheel || generator == &left_wheel_speed) {
    automaton = &left_trap;
    trap = &left_wheel;
    trap_speed = &left_wheel_speed;
  } else {
    return;
  }

  // Unregistering callbacks on the current ramp
  remove_callback(trap);
  remove_callback(trap_speed);

  // Automaton evolution
  switch (automaton->state) {
  case TRAPEZOID_STATE_STOP:
    // The automaton is already stopped, this shouldn't happen
    break;
  case TRAPEZOID_STATE_ACC:
    // Speed is inceasing, we have to go to the constant speed phase of the movement
    adjust_speed(trap_speed, 0.);
    adjust_value(trap_speed, automaton->speed);
    add_callback(trap, SELECT_THRESHOLD(automaton->dir), automaton->angle - automaton->speed*automaton->speed/automaton->acceleration/2., trapezoid_callback);
    automaton->state = TRAPEZOID_STATE_CONST;
    break;
  case TRAPEZOID_STATE_TRIANGLE:
    // Special case of triangle profile, we have to slow down at the end of the acceleration phase
    adjust_speed(trap_speed, -automaton->acceleration);
    adjust_value(trap_speed, automaton->speed);
    add_callback(trap, SELECT_THRESHOLD(automaton->dir), automaton->angle, trapezoid_callback);
    add_callback(trap_speed, SELECT_THRESHOLD_DEC(automaton->dir), 0.1*automaton->speed, trapezoid_callback);
    automaton->state = TRAPEZOID_STATE_DEC;
    break;
  case TRAPEZOID_STATE_CONST:
    // End of the constant speed phase, we have to slow down
    adjust_speed(trap_speed, -automaton->acceleration);
    add_callback(trap, SELECT_THRESHOLD(automaton->dir), automaton->angle, trapezoid_callback);
    add_callback(trap_speed, SELECT_THRESHOLD_DEC(automaton->dir), 0.1*automaton->speed, trapezoid_callback);
    automaton->state = TRAPEZOID_STATE_DEC;
    break;
  case TRAPEZOID_STATE_DEC:
    // End of the movement, stop the motor
    adjust_speed(trap_speed, 0.);
    adjust_value(trap_speed, 0.);
    //adjust_value(trap, automaton->angle);
    automaton->state = TRAPEZOID_STATE_STOP;
    break;
  }
}

void init_trajectory_controller(void) {
  float k[] = {-68.0325, -1.0205};
  float l0[] = {0.0236, 3.9715};

  // Create generator for position and speed manipulation
  new_ramp_generator(&right_wheel_speed, 0., 0.);
  new_ramp_generator(&left_wheel_speed, 0., 0.);
  new_ramp2_generator(&right_wheel, 0., &right_wheel_speed);
  new_ramp2_generator(&left_wheel, 0., &left_wheel_speed);

  // Start control of drive motors
  mc_new_controller(MOTOR3, ENCODER3, -360.0/2000.0/15.0, 0.833, 0.015, 0.005, k, -k[0], l0, &left_wheel);
  mc_new_controller(MOTOR4, ENCODER4, -360.0/2000.0/15.0, 0.833, 0.015, 0.005, k, -k[0], l0, &right_wheel);

  // Start the generator
  start_generator(&right_wheel);
  start_generator(&left_wheel);
  start_generator(&right_wheel_speed);
  start_generator(&left_wheel_speed);

  // Initial state of automatons
  right_trap.state = TRAPEZOID_STATE_STOP;
  left_trap.state = TRAPEZOID_STATE_STOP;
}

uint8_t tc_is_finished(void) {
  // Is there currently a trapezoidal command running ?
  if (right_trap.state == TRAPEZOID_STATE_STOP && left_trap.state == TRAPEZOID_STATE_STOP)
    return 1;

  // No running process, last planified action should be finished
  return 0;
}

void tc_move(float distance, float speed, float acceleration) {
  float acc_dist, t_acc, t_end;

  // Verify parameters
  if (distance == 0 || speed <= 0 || acceleration <= 0)
    return;

  // Compute some common parameters
  // For the right motor
  right_trap.init_val = get_output_value(&right_wheel);
  right_trap.dir = SIGN(distance);
  right_trap.angle = right_trap.init_val + distance / WHEEL_R * 180 / M_PI;
  right_trap.acceleration = right_trap.dir * acceleration / WHEEL_R * 180 / M_PI;
  // For the left motor
  left_trap.init_val = get_output_value(&left_wheel);
  left_trap.dir = SIGN(distance);
  left_trap.angle = left_trap.init_val + distance / WHEEL_R * 180 / M_PI;
  left_trap.acceleration = left_trap.dir * acceleration / WHEEL_R * 180 / M_PI;

  // Is the trapezoidal speed profile posible ?
  t_acc = speed / acceleration;
  t_end = (speed * speed + distance * acceleration) / (speed * acceleration);

  if (t_end > (2. * t_acc)) {
    // A trapezoidal speed profile is possible
    right_trap.is_triangle = 0;
    left_trap.is_triangle = 0;

    // Compute trapezoid parameters for right motor
    right_trap.speed = right_trap.dir * speed / WHEEL_R * 180 / M_PI;
    right_trap.state = TRAPEZOID_STATE_ACC;

    // Compute trapezoid parameters for left motor
    left_trap.speed = left_trap.dir * speed / WHEEL_R * 180 / M_PI;
    left_trap.state = TRAPEZOID_STATE_ACC;

    // This is distance during which the robot will accelerate
    acc_dist = SIGN(distance) * speed * speed / acceleration / 2.0 / WHEEL_R * 180.0 / M_PI;

    // Set accelerations for the trapezoid's first phase and associated callbacks
    adjust_speed(&right_wheel_speed, right_trap.acceleration);
    add_callback(&right_wheel, SELECT_THRESHOLD(right_trap.dir), right_trap.init_val + acc_dist, trapezoid_callback);
    add_callback(&right_wheel_speed, SELECT_THRESHOLD(right_trap.dir), right_trap.speed, trapezoid_callback);
    adjust_speed(&left_wheel_speed, left_trap.acceleration);
    add_callback(&left_wheel, SELECT_THRESHOLD(left_trap.dir), left_trap.init_val + acc_dist, trapezoid_callback);
    add_callback(&left_wheel_speed, SELECT_THRESHOLD(left_trap.dir), left_trap.speed, trapezoid_callback);
  } else {
    // A trapezoidal speed profile is not possible with the given acceleration, let's go for a triangle
    right_trap.is_triangle = 1;
    left_trap.is_triangle = 1;

    // Compute triangle parameters for right motor
    right_trap.speed = right_trap.dir * sqrt(right_trap.angle * right_trap.acceleration);
    right_trap.state = TRAPEZOID_STATE_TRIANGLE;

    // Compute triangle parameters for left motor
    left_trap.speed = left_trap.dir * sqrt(left_trap.angle * left_trap.acceleration);
    left_trap.state = TRAPEZOID_STATE_TRIANGLE;

    // Set accelerations for the triangle's first phase and associated callbacks
    adjust_speed(&right_wheel_speed, right_trap.acceleration);
    add_callback(&right_wheel, SELECT_THRESHOLD(right_trap.dir), right_trap.init_val + right_trap.angle/2.0, trapezoid_callback);
    add_callback(&right_wheel_speed, SELECT_THRESHOLD(right_trap.dir), right_trap.speed, trapezoid_callback);
    adjust_speed(&left_wheel_speed, left_trap.acceleration);
    add_callback(&left_wheel, SELECT_THRESHOLD(left_trap.dir), left_trap.init_val + left_trap.angle/2.0, trapezoid_callback);
    add_callback(&left_wheel_speed, SELECT_THRESHOLD(left_trap.dir), left_trap.speed, trapezoid_callback);
  }
}

void tc_turn(float angle, float speed, float acceleration) {
  float acc_angle, t_acc, t_end;

  // Verify parameters
  if (angle == 0 || speed <= 0 || acceleration <= 0)
    return;

  // Compute some common parameters
  // For the right motor
  right_trap.init_val = get_output_value(&right_wheel);
  right_trap.dir = SIGN(angle);
  right_trap.angle = right_trap.init_val + angle * STRUCT_B / 2.0 / WHEEL_R;
  right_trap.acceleration = right_trap.dir * acceleration * STRUCT_B / 2.0 / WHEEL_R;
  // For the left motor
  left_trap.init_val = get_output_value(&left_wheel);
  left_trap.dir = -right_trap.dir;
  left_trap.angle = left_trap.init_val - angle * STRUCT_B / 2.0 / WHEEL_R;
  left_trap.acceleration = left_trap.dir * acceleration * STRUCT_B / 2.0 / WHEEL_R;

  // Is the trapezoidal speed profile posible ?
  t_acc = speed / acceleration;
  t_end = (speed * speed + angle * acceleration) / (speed * acceleration);

  if (t_end > (2. * t_acc)) {
    // A trapezoidal speed profile is possible
    right_trap.is_triangle = 0;
    left_trap.is_triangle = 0;

    // Compute trapezoid parameters for right motor
    right_trap.speed = right_trap.dir * speed * STRUCT_B / 2.0 / WHEEL_R;
    right_trap.state = TRAPEZOID_STATE_ACC;

    // Compute trapezoid parameters for left motor
    left_trap.speed = left_trap.dir * speed * STRUCT_B / 2.0 / WHEEL_R;
    left_trap.state = TRAPEZOID_STATE_ACC;

    // This is the angle during which the robot will accelerate
    acc_angle = SIGN(angle) * speed * speed / acceleration / 2.0 * STRUCT_B / 2.0 / WHEEL_R;

    // Set accelerations for the trapezoid's first phase and associated callbacks
    adjust_speed(&right_wheel_speed, right_trap.acceleration);
    add_callback(&right_wheel, SELECT_THRESHOLD(right_trap.dir), right_trap.init_val + acc_angle, trapezoid_callback);
    add_callback(&right_wheel_speed, SELECT_THRESHOLD(right_trap.dir), right_trap.speed, trapezoid_callback);
    adjust_speed(&left_wheel_speed, left_trap.acceleration);
    add_callback(&left_wheel, SELECT_THRESHOLD(left_trap.dir), left_trap.init_val - acc_angle, trapezoid_callback);
    add_callback(&left_wheel_speed, SELECT_THRESHOLD(left_trap.dir), left_trap.speed, trapezoid_callback);
  } else {
    // A trapezoidal speed profile is not possible with the given acceleration, let's go for a triangle
    right_trap.is_triangle = 1;
    left_trap.is_triangle = 1;

    // Compute triangle parameters for right motor
    right_trap.speed = right_trap.dir * sqrt(right_trap.angle * right_trap.acceleration);
    right_trap.state = TRAPEZOID_STATE_TRIANGLE;

    // Compute triangle parameters for left motor
    left_trap.speed = left_trap.dir * sqrt(left_trap.angle * left_trap.acceleration);
    left_trap.state = TRAPEZOID_STATE_TRIANGLE;

    // Set accelerations for the triangle's first phase and associated callbacks
    adjust_speed(&right_wheel_speed, right_trap.acceleration);
    add_callback(&right_wheel, SELECT_THRESHOLD(right_trap.dir), right_trap.init_val + right_trap.angle/2.0, trapezoid_callback);
    add_callback(&right_wheel_speed, SELECT_THRESHOLD(right_trap.dir), right_trap.speed, trapezoid_callback);
    adjust_speed(&left_wheel_speed, left_trap.acceleration);
    add_callback(&left_wheel, SELECT_THRESHOLD(left_trap.dir), left_trap.init_val + left_trap.angle/2.0, trapezoid_callback);
    add_callback(&left_wheel_speed, SELECT_THRESHOLD(left_trap.dir), left_trap.speed, trapezoid_callback);
  }
}

