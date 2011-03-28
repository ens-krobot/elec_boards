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

// Robot parameters
#define WHEEL_R    0.049245
#define STRUCT_B   0.259

#include "motor_controller.h"
#include "command_generator.h"
#include <math.h>

/* Initialize the trajectory controller
 *
 * This function will initialize the trajectory controller and the
 * necessary motor controller.
 */
void init_trajectory_controller(void);

/* Indicates if the last planified action is finished
 *
 * Returns 1 if the last planified action is finished, 0 if it is not.
 */
uint8_t tc_is_finished(void);

/* Move along a line
 *  - distance : distance to move of (in meters), can be positive (forward)
 *                or negative (backward).
 *  - speed : moving speed (in meters per second) should be positive.
 *  - acceleration : in meters per second per second, should be positive.
 */
void tc_move(float distance, float speed, float acceleration);


/* Turn around the propulsion shaft center
 *  - angle : angle to turn of (in degrees), can be positive (CCW)
 *                or negative (CW).
 *  - speed : turning speed (in degrees per second) should be positive.
 *  - acceleration : in degrees per second per second, should be positive.
 */
void tc_turn(float angle, float speed, float acceleration);

#endif /* __TRAJECTORY_CONTROLLER_H */
