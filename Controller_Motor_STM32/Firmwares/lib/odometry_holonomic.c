/*
 * odometry_holonomic.c
 * --------------------
 * Copyright : (c) 2015, Xavier Lagorce <Xavier.Lagorce@crans.org>
 * Licence   : BSD3
 *
 * This file is a part of [kro]bot.
 */

#include "odometry_holonomic.h"

#define COS_FACT 0.6666666666666667f // 1/(1-cos(2*pi/3))
#define SIN_FACT 0.5773502691896257f // 1/(2*sin(2*pi/3))

#define ODOMETRY_STACK_SIZE (KERN_MINSTACKSIZE * 8)

PROC_DEFINE_STACK(stack_odometry, ODOMETRY_STACK_SIZE);

typedef struct {
  robot_state_t robot_state;
  float wheel_radius, drive_radius, encoder_gain;
  uint8_t front_encoder, back_left_encoder, back_right_encoder;
  float Ts;
  uint8_t enable;
  uint8_t running;
} odometry_state_t;

odometry_state_t state;

static void NORETURN odometry_process(void);

void HolonomicOdometryInit(float Ts,
                           float wheel_radius, float drive_radius,
                           uint8_t front_encoder, uint8_t back_left_encoder, uint8_t back_right_encoder,
                           float encoder_gain) {

  // Initialize initial state
  state.robot_state.x = 0.;
  state.robot_state.y = 0;
  state.robot_state.theta = 0;

  // Initialize robot parameters
  state.wheel_radius = wheel_radius;
  state.drive_radius = drive_radius;
  state.front_encoder = front_encoder;
  state.back_left_encoder = back_left_encoder;
  state.back_right_encoder = back_right_encoder;
  state.encoder_gain = encoder_gain;
  state.Ts = Ts;

  // Start odometry process
  state.enable = 1;
  proc_new(odometry_process, (&state), sizeof(stack_odometry), stack_odometry);
}

void HolOdo_disable(void) {
  state.enable = 0;
}

void HolOdo_restart(void) {
  // Start odometry process
  if (state.enable == 0 && state.running == 0) {
    state.enable = 1;
    proc_new(odometry_process, (&state), sizeof(stack_odometry), stack_odometry);
  }
}

static void NORETURN odometry_process(void) {
  float pos_f, pos_bl, pos_br, last_pos_f, last_pos_bl, last_pos_br, delta_f, delta_bl, delta_br, lx, ly;
  Timer timer;
  odometry_state_t *state;

  // get data
  state = (odometry_state_t *) proc_currentUserData();

  // configure timer
  timer_setDelay(&timer, us_to_ticks(1e6 * state->Ts));
  timer_setEvent(&timer);

  // Indicate we are running
  state->running = 1;

  // State initialization
  last_pos_f = (float)getEncoderPosition(state->front_encoder);
  last_pos_bl = (float)getEncoderPosition(state->back_left_encoder);
  last_pos_br = (float)getEncoderPosition(state->back_right_encoder);

  while (1) {
    if (state->enable == 0) {
      state->running = 0;
      proc_exit();
    } else {
      timer_add(&timer);

      // Measure motors rotation and correct wrapping
      pos_f = (float)getEncoderPosition(state->front_encoder);
      pos_bl = (float)getEncoderPosition(state->back_left_encoder);
      pos_br = (float)getEncoderPosition(state->back_right_encoder);
      delta_f = pos_f - last_pos_f;
      delta_bl = pos_bl - last_pos_bl;
      delta_br = pos_br - last_pos_br;
      if (delta_f > 32767) {
        delta_f = delta_f - 65535;
      } else if (delta_f < - 32767) {
        delta_f = delta_f + 65535;
      }
      if (delta_bl > 32767) {
        delta_bl = delta_bl - 65535;
      } else if (delta_bl < - 32767) {
        delta_bl = delta_bl + 65535;
      }
      if (delta_br > 32767) {
        delta_br = delta_br - 65535;
      } else if (delta_br < - 32767) {
        delta_br = delta_br + 65535;
      }
      delta_f *= state->encoder_gain;
      delta_bl *= state->encoder_gain;
      delta_br *= state->encoder_gain;
      last_pos_f = pos_f;
      last_pos_bl = pos_bl;
      last_pos_br = pos_br;

      // Displacement according to the robot's reference frame
      lx = (- delta_f
            + delta_bl / 2.f
            + delta_br / 2.f) * state->wheel_radius * COS_FACT;
      ly = (- delta_bl + delta_br) * state->wheel_radius * SIN_FACT;

      // New state computation
      state->robot_state.x += lx*cos(state->robot_state.theta) - ly*sin(state->robot_state.theta);
      state->robot_state.y += lx*sin(state->robot_state.theta) + ly*cos(state->robot_state.theta);
      state->robot_state.theta += (delta_f
                                   + delta_bl
                                   + delta_br) / (2.f*state->drive_radius) * state->wheel_radius * COS_FACT;

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

void HolOdo_setState(robot_state_t *new_state) {
  state.robot_state.x = new_state->x;
  state.robot_state.y = new_state->y;
  state.robot_state.theta = new_state->theta;
}

void HolOdo_getState(robot_state_t *robot_state) {
  if (state.enable == 1) {
    robot_state->x = state.robot_state.x;
    robot_state->y = state.robot_state.y;
    robot_state->theta = state.robot_state.theta;
  } else {
    robot_state->x = 0.f;
    robot_state->y = 0.f;
    robot_state->theta = 0.f;
  }
}

float HolOdo_getTheta(void) {
  return state.robot_state.theta;
}

