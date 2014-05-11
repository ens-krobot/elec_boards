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
  uint8_t enabled, tc_ind;
  int8_t direction;
  float bottom, extend;
  uint8_t end_stop_ind;
} lc_state_t;

lc_state_t lc_state[2];

float pos2angle(uint8_t lift, float position);

float pos2angle(uint8_t lift, float position) {

  //return lc_state[lift].offset - position / 23.474;
  return lc_state[lift].bottom + lc_state[lift].extend*position;
}

void lc_init(void) {
  motor_controller_params_t params;
  int i;

  // Init state
  for (i=0; i<2; i++) {
    lc_state[i].enabled = 1;
    lc_state[i].direction = -1;
    lc_state[i].bottom = 0;
    lc_state[i].extend = 0;
    lc_state[i].end_stop_ind = 0;
  }
  lc_state[LC_LEFT_LIFT].tc_ind = LC_TC_LEFT;
  lc_state[LC_RIGHT_LIFT].tc_ind = LC_TC_RIGHT;

  // Init Trajectory controllers
  tc_new_controller(LC_TC_LEFT);
  tc_new_controller(LC_TC_RIGHT);
  // Limit PWM value
  motorSetMaxPWM(MOTOR2, 1800); // Limit to 12V
  motorSetMaxPWM(MOTOR4, 1800); // Limit to 12V
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
  // Initialize left lift
  params.motor = MOTOR2;
  params.encoder = ENCODER2;
  mc_new_controller(&params, tc_get_position_generator(LC_TC_LEFT), CONTROLLER_MODE_NORMAL);
  // Initialize right lift
  params.motor = MOTOR4;
  params.encoder = ENCODER4;
  mc_new_controller(&params, tc_get_position_generator(LC_TC_RIGHT), CONTROLLER_MODE_NORMAL);
}

void lc_end_stop_reached(uint8_t end_stops) {

  // Set the end stop indicators and stop the lift if needed
  if ((end_stops & LC_LEFT_UP) || (end_stops & LC_LEFT_BOTTOM)) {
    if ((end_stops & LC_LEFT_UP) && (lc_state[LC_LEFT_LIFT].direction == 1))
      tc_stop(LC_TC_LEFT);
    if ((end_stops & LC_LEFT_BOTTOM) && (lc_state[LC_LEFT_LIFT].direction == -1))
      tc_stop(LC_TC_LEFT);
    lc_state[LC_LEFT_LIFT].end_stop_ind = end_stops & (LC_LEFT_UP|LC_LEFT_BOTTOM);
  }
  if ((end_stops & LC_RIGHT_UP) || (end_stops & LC_RIGHT_BOTTOM)) {
    if ((end_stops & LC_RIGHT_UP) && (lc_state[LC_RIGHT_LIFT].direction == 1))
      tc_stop(LC_TC_RIGHT);
    if ((end_stops & LC_RIGHT_BOTTOM) && (lc_state[LC_RIGHT_LIFT].direction == -1))
      tc_stop(LC_TC_RIGHT);
    lc_state[LC_RIGHT_LIFT].end_stop_ind = (end_stops & (LC_RIGHT_UP|LC_RIGHT_BOTTOM)) >> 2;
  }
}

void lc_homing(uint8_t lift) {
  // Go down
  lc_state[lift].direction = -1;
  tc_goto_speed(lc_state[lift].tc_ind, M_PI/4, 1);
  // Wait for the end stop to trigger
  while (!(lc_state[lift].end_stop_ind & LC_BOTTOM))
    cpu_relax();
  // Stop the lift
  tc_stop(lc_state[lift].tc_ind);
  // Assert end stop
  lc_state[lift].end_stop_ind &= ~LC_BOTTOM;
  // Wait and get current generator position as offset
  timer_delay(50);
  lc_state[lift].bottom = get_output_value(tc_get_position_generator(lc_state[lift].tc_ind));

  // Go up
  lc_state[lift].direction = 1;
  tc_goto_speed(lc_state[lift].tc_ind, -M_PI/4, 1);
  // Wait for the end stop to trigger
  while (!(lc_state[lift].end_stop_ind & LC_UP))
    cpu_relax();
  // Stop the lift
  tc_stop(lc_state[lift].tc_ind);
  // Assert end stop
  lc_state[lift].end_stop_ind &= ~LC_UP;
  // Wait and get current generator position as offset
  timer_delay(50);
  lc_state[lift].extend = get_output_value(tc_get_position_generator(lc_state[lift].tc_ind)) - lc_state[lift].bottom;
}

void lc_goto_position(uint8_t lift, float position) {
  float goal = pos2angle(lift, position);

  if (goal > get_output_value(tc_get_position_generator(lc_state[lift].tc_ind))) {
    lc_state[lift].direction = -1;
  } else {
    lc_state[lift].direction = 1;
  }
  tc_goto(lift, goal, M_PI, 5);
}

void lc_release(void) {
  mc_delete_controller(MOTOR2);
  mc_delete_controller(MOTOR4);

  lc_state[LC_LEFT_LIFT].enabled = 0;
  lc_state[LC_RIGHT_LIFT].enabled = 0;
}
