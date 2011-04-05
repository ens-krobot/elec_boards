/*
 * odometry.c
 * ----------
 * Copyright : (c) 2011, Xavier Lagorce <Xavier.Lagorce@crans.org>
 * Licence   : BSD3
 *
 * This file is a part of [kro]bot.
 */

#include "odometry.h"

PROC_DEFINE_STACK(stack_odometry, KERN_MINSTACKSIZE * 8);

typedef struct {
  robot_state_t robot_state;
  float wheel_radius, shaft_width, encoder_gain;
  float Ts;
  uint8_t enable;
} odometry_state_t;

odometry_state_t state;

static void NORETURN odometry_process(void);

void odometryInit(float Ts, float wheel_radius, float shaft_width, float encoder_gain) {

  // Initialize initial state
  state.robot_state.x = 0.;
  state.robot_state.y = 0;
  state.robot_state.theta = 0;

  // Initialize robot parameters
  state.wheel_radius = wheel_radius;
  state.shaft_width = shaft_width;
  state.encoder_gain = encoder_gain;
  state.Ts = Ts;

  // Start odometry process
  state.enable = 1;
  proc_new(odometry_process, NULL, sizeof(stack_odometry), stack_odometry);
}

void odo_disable(void) {
  state.enable = 0;
}

static void NORETURN odometry_process(void) {
  float pos_l, pos_r, last_pos_l, last_pos_r, delta_l, delta_r;
  uint8_t dir_l, dir_r;
  Timer timer;

  // configure timer
  timer_setDelay(&timer, ms_to_ticks(state.Ts));
  timer_setEvent(&timer);

  // State initialization
  last_pos_l = (float)getEncoderPosition(ENCODER3);
  last_pos_r = (float)getEncoderPosition(ENCODER4);

  while (1) {
    if (state.enable == 0) {
      proc_exit();
    } else {
      timer_add(&timer);

      // Measure motors rotation and correct wrapping
      pos_l = (float)getEncoderPosition(ENCODER3); dir_l = getEncoderDirection(ENCODER3);
      pos_r = (float)getEncoderPosition(ENCODER4); dir_r = getEncoderDirection(ENCODER4);
      delta_l = pos_l - last_pos_l;
      delta_r = pos_r - last_pos_r;
      if (dir_l  == ENCODER_DIR_UP) {
        if (delta_l < 0)
          delta_l = delta_l + 65535;
      } else {
        if (delta_l > 0)
          delta_l = delta_l - 65535;
      }
      if (dir_r  == ENCODER_DIR_UP) {
        if (delta_r < 0)
          delta_r = delta_r + 65535;
      } else {
        if (delta_r > 0)
          delta_r = delta_r - 65535;
      }
      delta_l *= state.encoder_gain;
      delta_r *= state.encoder_gain;
      last_pos_l = pos_l;
      last_pos_r = pos_r;

      // New state computation
      state.robot_state.x += state.wheel_radius * (delta_r + delta_l) / 2.0 * cos(state.robot_state.theta);
      state.robot_state.y += state.wheel_radius * (delta_r + delta_l) / 2.0 * sin(state.robot_state.theta);
      state.robot_state.theta += state.wheel_radius / state.shaft_width * (delta_r - delta_l);

      // Normalization of theta
      if (state.robot_state.theta > M_PI) {
        state.robot_state.theta -= 2*M_PI;
      } else if (state.robot_state.theta < -M_PI) {
        state.robot_state.theta += 2*M_PI;
      }
    }
    timer_waitEvent(&timer); // Wait for the remaining of the sample period
  }
}

void odo_setState(robot_state_t *new_state) {
  state.robot_state.x = new_state->x;
  state.robot_state.y = new_state->y;
  state.robot_state.theta = new_state->theta;
}

void odo_getState(robot_state_t *robot_state) {
  robot_state->x = state.robot_state.x;
  robot_state->y = state.robot_state.y;
  robot_state->theta = state.robot_state.theta;
}

