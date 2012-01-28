/*
 * arm2R_controller.c
 * -----------------
 * Copyright : (c) 2012, Xavier Lagorce <Xavier.Lagorce@crans.org>
 * Licence   : BSD3
 *
 * This file is a part of [kro]bot.
 */

#include "arm2R_controller.h"

typedef struct {
  uint8_t enabled, tc_ind[2];
  command_generator_t theta1_s, theta2_s, theta1, theta2;
} a2Rc_state_t;

a2Rc_state_t ac_state[2];

void a2Rc_init(void) {
  int i;

  // Init state
  for (i=0; i<2; i++) {
    ac_state[i].enabled = 0;
  }
  ac_state[ARM1].tc_ind[0] = A2R_ARM1_TC + A2R_TC_X;
  ac_state[ARM1].tc_ind[1] = A2R_ARM1_TC + A2R_TC_Y;
  ac_state[ARM2].tc_ind[0] = A2R_ARM2_TC + A2R_TC_X;
  ac_state[ARM2].tc_ind[1] = A2R_ARM2_TC + A2R_TC_Y;
}

void a2Rc_start(uint8_t arm,
                float l1, float l2,
                uint8_t enc_theta1, uint8_t enc_theta2) {

  uint8_t tc_x, tc_y;

  // Read trajectory controllers identifiers
  tc_x = ac_state[arm].tc_ind[0];
  tc_y = ac_state[arm].tc_ind[1];

  // Init Trajectory controllers
  tc_new_controller(tc_x);
  tc_new_controller(tc_y);
  new_a2r_generator(&ac_state[arm].theta1_s,
                    tc_get_position_generator(tc_x),
                    tc_get_speed_generator(tc_x),
                    tc_get_position_generator(tc_y),
                    tc_get_speed_generator(tc_y),
                    l1, l2, enc_theta1, enc_theta2, 1);
  new_a2r_generator(&ac_state[arm].theta2_s,
                    tc_get_position_generator(tc_x),
                    tc_get_speed_generator(tc_x),
                    tc_get_position_generator(tc_y),
                    tc_get_speed_generator(tc_y),
                    l1, l2, enc_theta1, enc_theta2, 2);
  new_ramp2_generator(&ac_state[arm].theta1, 0.0, &ac_state[arm].theta1_s);
  new_ramp2_generator(&ac_state[arm].theta2, 0.0, &ac_state[arm].theta2_s);

  start_generator(&ac_state[arm].theta1_s);
  start_generator(&ac_state[arm].theta2_s);
  start_generator(&ac_state[arm].theta1);
  start_generator(&ac_state[arm].theta2);
}

void a2Rc_goto_position_x(uint8_t arm, float position, float speed, float acc) {
  tc_goto(arm+A2R_TC_X, position, speed, acc);
}
void a2Rc_goto_position_y(uint8_t arm, float position, float speed, float acc) {
  tc_goto(arm+A2R_TC_Y, position, speed, acc);
}

command_generator_t* a2Rc_get_first_articulation_gen(uint8_t arm) {
  return &ac_state[arm].theta1;
}
command_generator_t* a2Rc_get_second_articulation_gen(uint8_t arm) {
  return &ac_state[arm].theta2;
}
