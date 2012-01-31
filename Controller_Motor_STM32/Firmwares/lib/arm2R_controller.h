/*
 * arm2R_controller.h
 * -----------------
 * Copyright : (c) 2012, Xavier Lagorce <Xavier.Lagorce@crans.org>
 * Licence   : BSD3
 *
 * This file is a part of [kro]bot.
 */

#ifndef __ARM2R_CONTROLLER_H
#define __ARM2R_CONTROLLER_H

#include "motor_controller.h"
#include "trajectory_controller.h"
#include "command_generator.h"

#define ARM1 0
#define ARM2 1

#ifndef A2R_ARM1_TC
  #define A2R_ARM1_TC 0
#endif
#ifndef A2R_ARM2_TC
  #define A2R_ARM2_TC 2
#endif

#define A2R_TC_X 0
#define A2R_TC_Y 1

/*
 * Initializes data structures associated with the arm 2R controller
 */
void a2Rc_init(void);

/*
 * Initializes a particular arm controller and configure the associated
 * generators
 *  - arm : ID of the arm controller to start
 *  - l1, l2 : length of the two segments of the arm
 *  - enc_theta1, enc_theta2 : IDs of the encoders measuring the position
 *                             of the two articulations
 */
void a2Rc_start(uint8_t arm,
                float l1, float l2,
                uint8_t enc_theta1, uint8_t enc_theta2);

/*
 * Moves an arm2R controller along an axis.
 *  - arm : ID of the arm to move
 *  - position : position on the axis to go to
 *  - speed : movement speed along the axis
 *  - acc : movement acceleration along the axis
 */
void a2Rc_goto_position_x(uint8_t arm, float position, float speed, float acc);
void a2Rc_goto_position_y(uint8_t arm, float position, float speed, float acc);

/*
 * Get the command generator outputing the reference position for
 * the first articulation
 *  - arm : ID of the arm of which to get the generator
 */
command_generator_t* a2Rc_get_first_articulation_gen(uint8_t arm);
/*
 * Get the command generator outputing the reference position for
 * the second articulation
 *  - arm : ID of the arm of which to get the generator
 */
command_generator_t* a2Rc_get_second_articulation_gen(uint8_t arm);

/*
 * Get the current angles of the articulations.
 *  - arm : ID of the arm of which to get the angles
 *  - theta1 : address of the variable to write the first angle into
 *  - theta2 : address of the variable to write the second angle into
 */
void a2Rc_get_angles(uint8_t arm, float *theta1, float *theta2);

/*
 * Computes the current position of the end of the arm
 *  - arm : ID of the arm of which to compute the end's position
 *  - x : address of the variable to wirte the X coordinate into
 *  - y : address of the variable to wirte the Y coordinate into
 */
void a2Rc_get_position(uint8_t arm, float *x, float *y);

#endif /* __ARM2R_CONTROLLER_H */
