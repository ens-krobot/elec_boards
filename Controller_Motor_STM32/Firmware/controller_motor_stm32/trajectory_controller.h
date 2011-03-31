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

typedef struct {
  uint8_t encoder;                  // Encoder ID to measure motor position from
  float encoder_gain;               // Gain to convert encoder value unit to reference unit
  float G0;                         // DC motor static gain
  float tau;                        // DC motor time constant
  float k[2];                       // State control gain
  float l;                          // Reference factoring gain
  float l0[2];                      // State observer gain
  float T;                          // Sampling period of the controller in seconds
} trajectory_controller_params_t;

typedef struct {
  uint8_t left_wheel;               // Left wheel motor ID
  uint8_t right_wheel;              // Right wheel motor ID
  float wheel_radius;               // Radius of the wheels
  float shaft_width;                // Width of the propulsion shaft
} tc_robot_t;

/* Initialize the trajectory controller system
 *
 */
void tc_init(void);

/* Initialize a new trajectory controller with given parameters
 *
 * This function will initialize a new trajectory controller and the
 * necessary motor controller and generators.
 */
void tc_new_controller(uint8_t motor, trajectory_controller_params_t *params);

/* Deletes a given trajectory controller
 *
 */
void tc_delete_controller(uint8_t motor);

/* Indicates if the last planified action on 'motors' is finished
 *
 * Returns the logical OR of working controllers among the ones specified in motors
 */
uint8_t tc_is_working(uint8_t motors);

/* Moves of a given distance
 *  - distance : distance to move of, can be positive (forward) or negative
 *               (backward). The units corresponds to the given scaling factor
 *               during initialization.
 *  - speed : moving speed (in meters per second) should be positive.
 *  - acceleration : in meters per second per second, should be positive.
 */
void tc_goto(uint8_t motor, float angle, float speed, float acceleration);

/* Accelerates or brakes the robot to the given speed
 *
 */
void tc_goto_speed(uint8_t motor, float speed, float acceleration);

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
