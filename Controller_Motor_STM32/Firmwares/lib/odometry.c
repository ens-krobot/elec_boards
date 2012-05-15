/*
 * odometry.c
 * ----------
 * Copyright : (c) 2011, Xavier Lagorce <Xavier.Lagorce@crans.org>
 * Licence   : BSD3
 *
 * This file is a part of [kro]bot.
 */

#include "odometry.h"

#define ODOMETRY_STACK_SIZE (KERN_MINSTACKSIZE * 8)
static cpu_stack_t stack_odometry[MAX_ODOMETRY_PROCESSES][(ODOMETRY_STACK_SIZE + sizeof(cpu_stack_t) - 1) / sizeof(cpu_stack_t)];

typedef struct {
  robot_state_t robot_state;
  float wheel_radius, shaft_width, left_encoder_gain, right_encoder_gain;
  float Ts;
  uint8_t enable;
  uint8_t running;
} odometry_state_t;

odometry_state_t state[MAX_ODOMETRY_PROCESSES];

static void NORETURN odometry_process(void);

void odometryInit(uint8_t process_num, float Ts, float wheel_radius, float shaft_width, float left_encoder_gain, float right_encoder_gain) {

  // Initialize initial state
  state[process_num].robot_state.x = 0.;
  state[process_num].robot_state.y = 0;
  state[process_num].robot_state.theta = 0;

  // Initialize robot parameters
  state[process_num].wheel_radius = wheel_radius;
  state[process_num].shaft_width = shaft_width;
  state[process_num].left_encoder_gain = left_encoder_gain;
  state[process_num].right_encoder_gain = right_encoder_gain;
  state[process_num].Ts = Ts;

  // Start odometry process
  state[process_num].enable = 1;
  proc_new(odometry_process, (&state[process_num]), sizeof(stack_odometry[process_num]), stack_odometry[process_num]);
}

void odo_disable(uint8_t process_num) {
  state[process_num].enable = 0;
}

void odo_restart(uint8_t process_num) {
  // Start odometry process
  if (state[process_num].enable == 0 && state[process_num].running == 0) {
    state[process_num].enable = 1;
    proc_new(odometry_process, (&state[process_num]), sizeof(stack_odometry[process_num]), stack_odometry[process_num]);
  }
}

static void NORETURN odometry_process(void) {
  float pos_l, pos_r, last_pos_l, last_pos_r, delta_l, delta_r;
  uint8_t dir_l, dir_r;
  Timer timer;
  odometry_state_t *state;

  // get data
  state = (odometry_state_t *) proc_currentUserData();

  // configure timer
  timer_setDelay(&timer, ms_to_ticks(1000 * state->Ts));
  timer_setEvent(&timer);

  // Indicate we are running
  state->running = 1;

  // State initialization
  last_pos_l = (float)getEncoderPosition(ENCODER3);
  last_pos_r = (float)getEncoderPosition(ENCODER4);

  while (1) {
    if (state->enable == 0) {
      state->running = 0;
      proc_exit();
    } else {
      timer_add(&timer);

      // Measure motors rotation and correct wrapping
      pos_l = (float)getEncoderPosition(ENCODER3); dir_l = getEncoderDirection(ENCODER3);
      pos_r = (float)getEncoderPosition(ENCODER4); dir_r = getEncoderDirection(ENCODER4);
      delta_l = pos_l - last_pos_l;
      delta_r = pos_r - last_pos_r;
      if (delta_l > 32767) {
        delta_l = delta_l - 65535;
      } else if (delta_l < - 32767) {
        delta_l = delta_l + 65535;
      }
      if (delta_r > 32767) {
        delta_r = delta_r - 65535;
      } else if (delta_r < - 32767) {
        delta_r = delta_r + 65535;
      }
      /*if (dir_l  == ENCODER_DIR_UP) {
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
          }*/
      delta_l *= state->left_encoder_gain;
      delta_r *= state->right_encoder_gain;
      last_pos_l = pos_l;
      last_pos_r = pos_r;

      // New state computation
      state->robot_state.x += state->wheel_radius * (delta_r + delta_l) / 2.0 * cos(state->robot_state.theta);
      state->robot_state.y += state->wheel_radius * (delta_r + delta_l) / 2.0 * sin(state->robot_state.theta);
      state->robot_state.theta += state->wheel_radius / state->shaft_width * (delta_r - delta_l);

      // Normalization of theta
      if (state->robot_state.theta > M_PI) {
        state->robot_state.theta -= 2*M_PI;
      } else if (state->robot_state.theta < -M_PI) {
        state->robot_state.theta += 2*M_PI;
      }
    }
    timer_waitEvent(&timer); // Wait for the remaining of the sample period
  }
}

void odo_setState(uint8_t process_num, robot_state_t *new_state) {
  state[process_num].robot_state.x = new_state->x;
  state[process_num].robot_state.y = new_state->y;
  state[process_num].robot_state.theta = new_state->theta;
}

void odo_getState(uint8_t process_num, robot_state_t *robot_state) {
  robot_state->x = state[process_num].robot_state.x;
  robot_state->y = state[process_num].robot_state.y;
  robot_state->theta = state[process_num].robot_state.theta;
}

