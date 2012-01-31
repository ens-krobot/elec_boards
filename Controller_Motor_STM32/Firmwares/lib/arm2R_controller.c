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
  uint8_t enabled, tc_ind[2], enc_theta1, enc_theta2;
  command_generator_t theta1_s, theta2_s, theta1, theta2;
  float l1, l2;
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
  float theta1, theta2, x, y;

  // Record the parameters of the arm
  ac_state[arm].l1 = l1;
  ac_state[arm].l2 = l2;
  ac_state[arm].enc_theta1 = enc_theta1;
  ac_state[arm].enc_theta2 = enc_theta2;

  // Get the current position of the arm
  a2Rc_get_angles(arm, &theta1, &theta2);
  x = l1*cos(theta1) + l2*cos(theta2);
  y = l1*sin(theta1) + l2*sin(theta2);

  // Read trajectory controllers identifiers
  tc_x = ac_state[arm].tc_ind[0];
  tc_y = ac_state[arm].tc_ind[1];

  // Init Trajectory controllers
  tc_new_controller(tc_x);
  tc_new_controller(tc_y);
  tc_set_position(tc_x, x);
  tc_set_position(tc_y, y);
  // Create the command generators
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
  // Create the reference generators
  new_ramp2_generator(&ac_state[arm].theta1, theta1, &ac_state[arm].theta1_s);
  new_ramp2_generator(&ac_state[arm].theta2, theta2, &ac_state[arm].theta2_s);

  // Start the different references generators
  start_generator(&ac_state[arm].theta1_s);
  start_generator(&ac_state[arm].theta2_s);
  start_generator(&ac_state[arm].theta1);
  start_generator(&ac_state[arm].theta2);

  // Activate the 'enabled' flag
  ac_state[arm].enabled = 1;
}

void a2Rc_goto_position_x(uint8_t arm, float position, float speed, float acc) {
  tc_goto(arm+A2R_TC_X, position, speed, acc);
}
void a2Rc_goto_position_y(uint8_t arm, float position, float speed, float acc) {
  tc_goto(arm+A2R_TC_Y, position, speed, acc);
}

uint8_t a2Rc_is_moving(uint8_t arm) {
  return tc_is_working(TC_MASK(ac_state[arm].tc_ind[0]))
    || tc_is_working(TC_MASK(ac_state[arm].tc_ind[1]));
}


command_generator_t* a2Rc_get_first_articulation_gen(uint8_t arm) {
  return &ac_state[arm].theta1;
}
command_generator_t* a2Rc_get_second_articulation_gen(uint8_t arm) {
  return &ac_state[arm].theta2;
}

void a2Rc_get_angles(uint8_t arm, float *theta1, float *theta2) {
  *theta1 = getEncoderPosition_f(ac_state[arm].enc_theta1);
  *theta2 = getEncoderPosition_f(ac_state[arm].enc_theta2);
}

void a2Rc_get_position(uint8_t arm, float *x, float *y) {
  float theta1, theta2;

  a2Rc_get_angles(arm, &theta1, &theta2);
  *x = ac_state[arm].l1 * cos(theta1) + ac_state[arm].l2 * cos(theta2);
  *y = ac_state[arm].l1 * sin(theta1) + ac_state[arm].l2 * sin(theta2);
}
