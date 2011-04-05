/*
 * differential_drive.c
 * --------------------
 * Copyright : (c) 2011, Xavier Lagorce <Xavier.Lagorce@crans.org>
 * Licence   : BSD3
 *
 * This file is a part of [kro]bot.
 */

#include "differential_drive.h"

typedef struct {
  uint8_t initialized, enabled;
  float wheel_radius, shaft_radius;
  float last_lin_acceleration, last_rot_acceleration;
  command_generator_t left_wheel_speed, right_wheel_speed;
  command_generator_t left_wheel, right_wheel;
} dd_params_t;

static dd_params_t params;

void dd_start(float wheel_radius, float shaft_width, float max_wheel_speed) {
  params.wheel_radius = wheel_radius;
  params.shaft_radius = shaft_width;
  params.last_lin_acceleration = 0.0;
  params.last_rot_acceleration = 0.0;

  tc_new_controller(DD_LINEAR_SPEED_TC);
  tc_new_controller(DD_ROTATIONAL_SPEED_TC);
  new_dd_generator(&params.left_wheel_speed,
                   tc_get_position_generator(DD_LINEAR_SPEED_TC),
                   tc_get_speed_generator(DD_LINEAR_SPEED_TC),
                   tc_get_position_generator(DD_ROTATIONAL_SPEED_TC),
                   tc_get_speed_generator(DD_ROTATIONAL_SPEED_TC),
                   wheel_radius, shaft_width, max_wheel_speed,
                   -1);
  new_dd_generator(&params.right_wheel_speed,
                   tc_get_position_generator(DD_LINEAR_SPEED_TC),
                   tc_get_speed_generator(DD_LINEAR_SPEED_TC),
                   tc_get_position_generator(DD_ROTATIONAL_SPEED_TC),
                   tc_get_speed_generator(DD_ROTATIONAL_SPEED_TC),
                   wheel_radius, shaft_width, max_wheel_speed,
                   1);
  new_ramp2_generator(&params.left_wheel, 0.0, &params.left_wheel_speed);
  new_ramp2_generator(&params.right_wheel, 0.0, &params.right_wheel_speed);

  start_generator(&params.left_wheel_speed);
  start_generator(&params.right_wheel_speed);
  start_generator(&params.left_wheel);
  start_generator(&params.right_wheel);

  params.initialized = 1;
  params.enabled = 1;
}

void dd_pause(void) {
  if (params.initialized) {
    dd_set_linear_speed(0.0, params.last_lin_acceleration);
    dd_set_rotational_speed(0.0, params.last_rot_acceleration);
    params.enabled = 0;
  }
}

void dd_resume(void) {
  if (params.initialized && params.enabled) {
    params.enabled = 1;
  }
}

void dd_stop(void) {
  if (params.initialized) {
    pause_generator(&params.left_wheel);
    pause_generator(&params.right_wheel);
    pause_generator(&params.left_wheel_speed);
    pause_generator(&params.right_wheel_speed);
    tc_delete_controller(DD_LINEAR_SPEED_TC);
    tc_delete_controller(DD_ROTATIONAL_SPEED_TC);
    params.enabled = 0;
    params.initialized = 0;
  }
}

command_generator_t* dd_get_left_wheel_generator(void) {
  return &params.left_wheel;
}

command_generator_t* dd_get_right_wheel_generator(void) {
  return &params.right_wheel;
}

void dd_move(float distance, float speed, float acceleration) {
  if (params.enabled) {
    params.last_lin_acceleration = acceleration;
    tc_goto(DD_LINEAR_SPEED_TC, distance, speed, params.last_lin_acceleration);
  }
}

void dd_turn(float angle, float speed, float acceleration) {
  if (params.enabled) {
    params.last_rot_acceleration = acceleration;
    tc_goto(DD_ROTATIONAL_SPEED_TC, angle, speed, params.last_rot_acceleration);
  }
}

void dd_set_linear_speed(float speed, float acceleration) {
  if (params.enabled) {
    params.last_lin_acceleration = acceleration;
    tc_goto_speed(DD_LINEAR_SPEED_TC, speed, params.last_lin_acceleration);
  }
}

void dd_set_rotational_speed(float speed, float acceleration) {
  if (params.enabled) {
    params.last_rot_acceleration = acceleration;
    tc_goto_speed(DD_ROTATIONAL_SPEED_TC, speed, params.last_rot_acceleration);
  }
}

