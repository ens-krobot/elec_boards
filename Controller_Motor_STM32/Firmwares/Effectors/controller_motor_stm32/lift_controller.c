/*
 * lift_controller.c
 * -----------------
 * Copyright : (c) 2011, Xavier Lagorce <Xavier.Lagorce@crans.org>
 * Licence   : BSD3
 *
 * This file is a part of [kro]bot.
 */

#include "lift_controller.h"

#define DIR_UP 0
#define DIR_DOWN 1

typedef struct {
  uint8_t enabled;
  int8_t direction_front, direction_back;
  float offset_front, offset_back;
  uint8_t end_stop_ind;
} lc_state_t;

lc_state_t lc_state;

float pos2angle(uint8_t lift, float position);
float pos2angle(uint8_t lift, float position) {
  float offset;

  switch(lift) {
  case LC_FRONT_LIFT:
    offset = lc_state.offset_front;
    break;
  case LC_BACK_LIFT:
    offset = lc_state.offset_back;
  }

  return offset - position / 23.474;
}

void lc_init(void) {
  motor_controller_params_t params;

  // Init state
  lc_state.enabled = 1;
  lc_state.direction_front = -1;
  lc_state.direction_back = -1;
  lc_state.offset_front = 0;
  lc_state.offset_back = 0;
  lc_state.end_stop_ind = 0;

  // Init Trajectory controllers
  tc_new_controller(LC_TC_FRONT);
  tc_new_controller(LC_TC_BACK);
  // Limit PWM value
  motorSetMaxPWM(MOTOR3, 1600);
  motorSetMaxPWM(MOTOR4, 1600);
  // Common parameters
  params.encoder_gain = 2.0*M_PI/588.0;
  params.G0 = 0.0035;
  params.tau = 0.025;
  params.k[0] = -10216;
  params.k[1] = -255.39;
  params.l = -params.k[0];
  params.l0[0] = 0.0091;
  params.l0[1] = 1.6361;
  params.T = 0.005;
  // Initialize front lift
  params.motor = MOTOR4;
  params.encoder = ENCODER4;
  mc_new_controller(&params, tc_get_position_generator(LC_TC_FRONT), CONTROLLER_MODE_NORMAL);
  // Initialize back lift
  params.motor = MOTOR3;
  params.encoder = ENCODER3;
  mc_new_controller(&params, tc_get_position_generator(LC_TC_BACK), CONTROLLER_MODE_NORMAL);
}

void lc_end_stop_reached(uint8_t end_stops) {

  // Stop the lift if needed
  if ((end_stops & LC_FRONT_UP) && (lc_state.direction_front == 1))
    tc_stop(LC_TC_FRONT);
  if ((end_stops & LC_FRONT_BOTTOM) && (lc_state.direction_front == -1))
    tc_stop(LC_TC_FRONT);
  if ((end_stops & LC_BACK_UP) && (lc_state.direction_back == 1))
    tc_stop(LC_TC_BACK);
  if ((end_stops & LC_BACK_BOTTOM) && (lc_state.direction_back == -1))
    tc_stop(LC_TC_BACK);

  lc_state.end_stop_ind = end_stops;
}

void lc_search_zero(uint8_t lift) {
  uint8_t tc_ind, end_stop;

  switch(lift) {
  case LC_FRONT_LIFT:
    tc_ind = LC_TC_FRONT;
    end_stop = LC_FRONT_BOTTOM;
    break;
  case LC_BACK_LIFT:
    tc_ind = LC_TC_BACK;
    end_stop = LC_BACK_BOTTOM;
    break;
  default:
    return;
  }

  // Go down
  tc_goto_speed(tc_ind, M_PI/4, 1);
  // Wait for the end stop to trigger
  while (!(lc_state.end_stop_ind & end_stop))
    cpu_relax();
  // Stop the lift
  tc_stop(tc_ind);
  // Assert end stop
  lc_state.end_stop_ind &= ~end_stop;
  // Wait and get current generator position as offset
  timer_delay(50);
  switch(lift) {
  case LC_FRONT_LIFT:
    lc_state.offset_front = get_output_value(tc_get_position_generator(tc_ind));
    break;
  case LC_BACK_LIFT:
    lc_state.offset_back = get_output_value(tc_get_position_generator(tc_ind));
    break;
  }
}

void lc_goto_position(uint8_t lift, float position) {
  float goal = pos2angle(lift, position);
  uint8_t tc_ind;

  switch(lift) {
  case LC_FRONT_LIFT:
    tc_ind = LC_TC_FRONT;
    break;
  case LC_BACK_LIFT:
    tc_ind = LC_TC_BACK;
    break;
  default:
    return;
  }

  if (goal > get_output_value(tc_get_position_generator(tc_ind))) {
    if (lift == LC_FRONT_LIFT) {
      lc_state.direction_front = -1;
    }
    else {
      lc_state.direction_back = -1;
    }
  } else {
    if (lift == LC_FRONT_LIFT) {
      lc_state.direction_front = 1;
    }
    else {
      lc_state.direction_back = 1;
    }
  }
  tc_goto(lift, pos2angle(lift, position), M_PI, 5);
}

void lc_release(void) {
  mc_delete_controller(MOTOR3);
  mc_delete_controller(MOTOR4);

  lc_state.enabled = 0;
}
