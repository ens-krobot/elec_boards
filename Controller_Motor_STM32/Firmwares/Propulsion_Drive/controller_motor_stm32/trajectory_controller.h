/*
 * trajectory_controller.h
 * -----------------------
 * Copyright : (c) 2011, Xavier Lagorce <Xavier.Lagorce@crans.org>
 * Licence   : BSD3
 *
 * This file is a part of [kro]bot.
 */

#ifndef __TRAJECTORY_CONTROLLER_H
#define __TRAJECTORY_CONTROLLER_H

#include "motor_controller.h"
#include "command_generator.h"
#include <math.h>

#define NUM_TC_MAX 4
#define TC_MASK(num) (1 << (num))

typedef struct {
  uint8_t left_wheel;               // Left wheel trajectory controller index
  uint8_t right_wheel;              // Right wheel trajectory controller index
  float wheel_radius;               // Radius of the wheels
  float shaft_width;                // Width of the propulsion shaft
} tc_robot_t;

/*
 * Initialize the trajectory controller system
 */
void tc_init(void);

/* Initialize a new trajectory controller with given parameters
 *
 * This function will initialize a new trajectory controller
 *  - cntr_index : index of the trajectory controller to initialize. It should
 *                 be between 0 and (NUM_TC_MAX-1)
 */
void tc_new_controller(uint8_t cntr_index);

/*
 * Deletes a given trajectory controller
 */
void tc_delete_controller(uint8_t cntr_index);

/* Indicates if the last planified action on 'cntr_indexes' is finished
 *  - cntr_indexes : logical OR of TC_MASK(cntr_index) for interesting indexes
 *
 * Returns the logical OR of working controllers among the ones specified in motors
 */
uint8_t tc_is_working(uint8_t cntr_indexes);

/*
 * Returns the position generator outputting the result of the 'cntr_index'th
 *  Trajectory Generator.
 */
command_generator_t* tc_get_position_generator(uint8_t cntr_index);

/*
 * Returns the speed generator outputting the intermediate result of the 'cntr_index'th
 *  Trajectory Generator.
 */
command_generator_t* tc_get_speed_generator(uint8_t cntr_index);


/* Moves of a given angle
 *  - angle : angle to move of, can be positive (forward) or negative
 *            (backward). The units corresponds to the given scaling factor
 *            during initialization.
 *  - speed : moving speed (in units per second) should be positive.
 *  - acceleration : in units per second per second, should be positive.
 */
void tc_goto(uint8_t cntr_index, float angle, float speed, float acceleration);

/* Accelerates or brakes the robot to the given speed
 *
 */
void tc_goto_speed(uint8_t cntr_index, float speed, float acceleration);

/*
 * Change the speed command of a controller without any concern about acceleration
 */
void tc_set_speed(uint8_t cntr_index, float speed);

/* Moves along a line
 *  - robot : pointer to a structure describing the robot configuration
 *  - distance : distance to move of (in meters), can be positive (forward)
 *                or negative (backward).
 *  - speed : moving speed (in meters per second) should be positive.
 *  - acceleration : in meters per second per second, should be positive.
 */
void tc_move(tc_robot_t *robot, float distance, float speed, float acceleration);

/* Turns around the propulsion shaft center
 *  - robot : pointer to a structure describing the robot configuration
 *  - angle : angle to turn of (in degrees), can be positive (CCW)
 *                or negative (CW).
 *  - speed : turning speed (in degrees per second) should be positive.
 *  - acceleration : in degrees per second per second, should be positive.
 */
void tc_turn(tc_robot_t *robot, float angle, float speed, float acceleration);

#endif /* __TRAJECTORY_CONTROLLER_H */
