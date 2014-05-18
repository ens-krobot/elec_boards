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

PROC_DEFINE_STACK(stack_homing, KERN_MINSTACKSIZE * 2);

typedef struct {
  uint8_t enabled, tc_ind;
  int8_t direction;
  float bottom, extend;
  uint8_t end_stop_ind;
  uint8_t homing_done, should_home, homing_state;
  float homing_speed;
} lc_state_t;

lc_state_t lc_state[2];

float pos2angle(uint8_t lift, float position);

static void NORETURN homing_process(void)
{
  while(1) {
    for (int i=0; i < 2; i++) {
      switch (lc_state[i].homing_state) {
      case 0:
        if (lc_state[i].should_home != 0) {
          lc_state[i].homing_done = 0;
          // Assert end stop
          lc_state[i].end_stop_ind &= ~LC_BOTTOM;
          lc_state[i].homing_state = 1;
          lc_state[i].should_home = 0;
        }
        break;
      case 1:
          // Go down if necessary
        if (!(lc_state[i].end_stop_ind & LC_BOTTOM)) {
          lc_state[i].direction = -1;
          tc_goto_speed(lc_state[i].tc_ind, -lc_state[i].homing_speed, 1);
        }
        lc_state[i].homing_state = 2;
        break;
      case 2:
        if (lc_state[i].end_stop_ind & LC_BOTTOM) {
          // Stop the lift
          tc_stop(lc_state[i].tc_ind);
          // Assert end stop
          lc_state[i].end_stop_ind &= ~LC_BOTTOM;
          lc_state[i].homing_state = 3;
        }
        break;
      case 3:
        lc_state[i].bottom = 0.;
        if (i == LC_LEFT_LIFT) {
          mc_suspend_controller(MOTOR3);
          tc_set_position(lc_state[i].tc_ind, 0.);
          mc_reactivate_controller(MOTOR3);
        } else if (i == LC_RIGHT_LIFT) {
          mc_suspend_controller(MOTOR4);
          tc_set_position(lc_state[i].tc_ind, 0.);
          mc_reactivate_controller(MOTOR4);
        }
        lc_state[i].homing_done = 1;
        lc_state[i].homing_state = 0;
        break;
      default:
        lc_state[i].homing_state = 0;
        break;
      }
    }
    timer_delay(20);
  }
}

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
    lc_state[i].extend = 1. / 0.005;
    lc_state[i].homing_done = 0;
    lc_state[i].should_home = 0;
    lc_state[i].homing_state = 0;
    lc_state[i].homing_speed = M_PI/4;
  }
  lc_state[LC_LEFT_LIFT].tc_ind = LC_TC_LEFT;
  lc_state[LC_RIGHT_LIFT].tc_ind = LC_TC_RIGHT;

  // Init Trajectory controllers
  tc_new_controller(LC_TC_LEFT);
  tc_new_controller(LC_TC_RIGHT);
  // Limit PWM value
  motorSetMaxPWM(MOTOR3, 1800); // Limit to 12V
  motorSetMaxPWM(MOTOR4, 1800); // Limit to 12V
  // Invert Right lift motor orientation
  motorInvertDirection(MOTOR4, 1);
  // Common parameters
  params.encoder_gain = -2.0*M_PI/360.0;
  params.G0 = 0.0035;
  params.tau = 0.025;
  params.k[0] = -10216;
  params.k[1] = -255.39;
  params.l = -params.k[0];
  params.l0[0] = 0.0091;
  params.l0[1] = 1.6361;
  params.T = 0.005;
  // Initialize left lift
  params.motor = MOTOR3;
  params.encoder = ENCODER2;
  mc_new_controller(&params, tc_get_position_generator(LC_TC_LEFT), CONTROLLER_MODE_NORMAL);
  // Initialize right lift
  params.motor = MOTOR4;
  params.encoder = ENCODER4;
  params.encoder_gain = 2.0*M_PI/360.0;
  mc_new_controller(&params, tc_get_position_generator(LC_TC_RIGHT), CONTROLLER_MODE_NORMAL);

  // start homing thread
  proc_new(homing_process, NULL, sizeof(stack_homing), stack_homing);
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

void lc_homing(uint8_t lift, float speed) {
  if (speed == 0) {
    speed = M_PI/4;
  } else if (speed < 0) {
    speed = -speed;
  }

  if (lc_state[lift].should_home == 0 &&
      lc_state[lift].homing_state == 0) {
    lc_state[lift].homing_speed = speed;
    lc_state[lift].should_home = 1;
  }
  /*// Assert end stop
  lc_state[lift].end_stop_ind &= ~LC_BOTTOM;
  // Go down
  lc_state[lift].direction = -1;
  tc_goto_speed(lc_state[lift].tc_ind, -speed, 1);
  // Wait for the end stop to trigger
  while (!(lc_state[lift].end_stop_ind & LC_BOTTOM))
    cpu_relax();
  // Stop the lift
  tc_stop(lc_state[lift].tc_ind);
  // Assert end stop
  lc_state[lift].end_stop_ind &= ~LC_BOTTOM;
  // Wait and get current generator position as offset
  timer_delay(50);
  //lc_state[lift].bottom = get_output_value(tc_get_position_generator(lc_state[lift].tc_ind));
  lc_state[lift].bottom = 0.;
  if (lift == LC_LEFT_LIFT) {
    mc_suspend_controller(MOTOR3);
    tc_set_position(lc_state[lift].tc_ind, 0.);
    mc_reactivate_controller(MOTOR3);
  } else if (lift == LC_RIGHT_LIFT) {
    mc_suspend_controller(MOTOR4);
    tc_set_position(lc_state[lift].tc_ind, 0.);
    mc_reactivate_controller(MOTOR4);
    }*/

  /*// Go up
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
  */
}

uint8_t is_homing_finished(uint8_t lift) {
  if (lc_state[lift].should_home == 0 &&
      lc_state[lift].homing_state == 0 &&
      lc_state[lift].homing_done == 1) {
    return 1;
  } else {
    return 0;
  }
}

float lc_get_position(uint8_t lift) {
  return get_output_value(tc_get_position_generator(lc_state[lift].tc_ind));
}

void lc_goto_position(uint8_t lift, float position) {
  float goal = pos2angle(lift, position);

  if (lc_state[lift].homing_done == 1) {
    if (goal > get_output_value(tc_get_position_generator(lc_state[lift].tc_ind))) {
      lc_state[lift].direction = 1;
    } else {
      lc_state[lift].direction = -1;
    }
    tc_goto(lc_state[lift].tc_ind, goal, M_PI, 5);
  }
}

void lc_release(void) {
  mc_delete_controller(MOTOR3);
  mc_delete_controller(MOTOR4);

  lc_state[LC_LEFT_LIFT].enabled = 0;
  lc_state[LC_RIGHT_LIFT].enabled = 0;
}
